#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/components/speaker/speaker.h"

#include <cstdint>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace esphome {
namespace va_client {

class VaClient;

// Triggers live as nested-style siblings of VaClient — declared here so the
// schema-side codegen can reference the same fully-qualified names. They
// register themselves with the parent via add_*_trigger() so the yaml-
// generated trigger lifecycle stays standard.

class OnPhaseTrigger : public Trigger<std::string> {
 public:
  explicit OnPhaseTrigger(VaClient *parent);
};

class OnRepeatedFailureTrigger : public Trigger<> {
 public:
  explicit OnRepeatedFailureTrigger(VaClient *parent);
};

// Fires when the device opens a follow-up mic window (i.e. server's
// request_follow_up message landed and the audio buffer has drained).
// yaml uses this to play the wake chime + flip the LED to "listening"
// so the user knows the assistant is waiting for their answer.
class OnFollowupOpenedTrigger : public Trigger<> {
 public:
  explicit OnFollowupOpenedTrigger(VaClient *parent);
};

class VaClient : public Component {
 public:
  void set_url(const std::string &url) { url_ = url; }
  void set_token(const std::string &token) { token_ = token; }
  void set_microphone(microphone::Microphone *m) { mic_ = m; }
  void set_mic_channel(uint8_t c) { mic_channel_ = c; }
  void set_speaker(speaker::Speaker *s) { speaker_ = s; }
  // Sets the output-volume multiplier applied to TTS in handle_binary_.
  // Driven from yaml by external_media_player's volume / mute state so the
  // device's physical +/- buttons and mute switch scale our TTS the same
  // way they scale chime announcements played through media_player. (No
  // HA api: block on this firmware — there's no remote slider.) Range
  // [0, 1]; values are clamped on read so callers don't have to bounds-check.
  void set_volume(float v) { volume_ = v; }
  void add_on_phase_trigger(OnPhaseTrigger *t) { phase_triggers_.push_back(t); }
  void add_on_repeated_failure_trigger(OnRepeatedFailureTrigger *t) {
    repeated_failure_triggers_.push_back(t);
  }
  void add_on_followup_opened_trigger(OnFollowupOpenedTrigger *t) {
    followup_opened_triggers_.push_back(t);
  }

  bool is_connected() const { return ws_connected_; }

  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // YAML-callable actions.
  void start_session();
  void send_interrupt();
  // Drop any TTS audio still queued in the PSRAM playback ring (head/tail/
  // fill = 0). Whatever already reached the downstream resampler/mixer leaf
  // still drains (~600 ms residual); the yaml stops that chain explicitly.
  // Called from yaml at barge-in (wake word during a reply) so the old
  // reply's queued tail can't keep draining under the wake chime before
  // start_session() runs, and folded into start_session()/send_interrupt()
  // as a backstop. Safe to call when the ring is already empty (no-op).
  void flush_audio_queue();
  // Called from yaml's on_followup_opened automation AFTER the chime has
  // finished announcing through the speaker (wait_until !is_announcing +
  // i2s tail). Opens the mic for kRequestFollowUpMs. No-op if the device
  // is no longer armed (e.g. user already pressed wake before the chime
  // finished — the new session takes priority).
  void commit_followup_mic();

  // Called from the static esp-idf event handler trampoline.
  void on_ws_event(int32_t event_id, void *event_data);

 protected:
  // Internal state machine. One canonical source of truth for the
  // bridge's lifecycle; everything else (mic gating, LED emission, drain
  // logic, timer ownership) derives from it. Declared up here because
  // method declarations below take State as a parameter — C++ resolves
  // class-scoped types in declaration order, so the enum must come
  // first.
  enum class State : uint8_t {
    Idle,           // bridge idle, mic off, no audio queued
    Listening,      // mic streaming up; pre- and post-server-VAD confirm
    Thinking,       // server processing (incl. tool calls); mic off
    Replying,       // TTS audio coming down; mic off
    WaitingDrain,   // server said idle; we're waiting for the ring +
                    //   speaker chain to actually play out before
                    //   emitting LED-idle and (maybe) opening followup
    FollowupArmed,  // request_follow_up handoff: yaml is playing the
                    //   chime; commit_followup_mic() will transition
                    //   us back to Listening when the chime ends
  };

  void connect_();
  void schedule_reconnect_();
  void on_mic_data_(const std::vector<uint8_t> &samples);
  void handle_text_(const char *data, size_t len);
  void handle_binary_(const uint8_t *data, size_t len);
  // Send {"type":"start"} — the "a turn is beginning" signal. Sent from
  // start_session() (wake word), so the bridge can flip to the listening
  // phase immediately instead of waiting for OpenAI's server-VAD
  // speech_started. Also covers barge-in: the bridge cancels any reply still
  // in flight on `start`, so we don't send a separate interrupt to barge.
  // No-op if the WS isn't connected.
  void send_start_();
  // Move the state machine to `next` and emit a phase LED transition to
  // `phase_label` (the user-visible name passed to yaml triggers). Stamps
  // state_entered_ms_ so state-bound timers can reference it.
  void transition_(State next, const std::string &phase_label);
  // Apply the server's reported phase ("listening" | "thinking" | "replying"
  // | "idle") to the state machine. Mostly a wrapper around transition_
  // with the policy choices (when to defer LED-idle, when to open follow-
  // up, etc.) collected in one place.
  void apply_server_phase_(const std::string &phase);
  // Returns true if the mic should be forwarding frames to the server.
  // Single derivation from current_state_ — no separate streaming flag.
  bool is_mic_streaming_() const;
  // Fire the on_phase trigger from the main loop. transition_ may be
  // called from the WS task; ESPHome triggers aren't thread-safe so we
  // marshal the actual trigger fire onto the main loop via defer().
  void emit_phase_(const std::string &phase);
  // Called once the speaker chain has actually drained (or kSpeakerStopTimeoutMs
  // elapsed). Decides whether to open a follow-up window or go straight to Idle.
  void finish_drain_();

  std::string url_;
  std::string token_;
  // Lifetime-stable storage referenced by esp_websocket_client_config_t.headers.
  std::string auth_header_;
  uint8_t mic_channel_{0};

  microphone::Microphone *mic_{nullptr};
  speaker::Speaker *speaker_{nullptr};

  // esp_websocket_client_handle_t kept opaque to avoid leaking esp-idf into the header.
  void *ws_handle_{nullptr};
  bool ws_connected_{false};

  uint32_t reconnect_delay_ms_{1000};
  // Set when a reconnect timer is in flight. esp_websocket_client emits both
  // DISCONNECTED and CLOSED (and sometimes ERROR) per failure; without this
  // guard we'd double-bump the backoff delay and double-log.
  bool reconnect_pending_{false};

  State current_state_{State::Idle};
  uint32_t state_entered_ms_{0};

  // Set during Replying when the server sends {"type":"request_follow_up"}.
  // Consumed on Replying → WaitingDrain → FollowupArmed to use the longer
  // mic window (kRequestFollowUpMs) and fire on_followup_opened so yaml
  // plays the question chime.
  bool request_follow_up_for_next_turn_{false};
  // Set by send_interrupt(). Consumed on the next server phase=idle so
  // it routes WaitingDrain → Idle (no chime, no mic) instead of opening
  // a follow-up window the user explicitly cancelled.
  bool interrupt_pending_{false};

  std::string current_phase_{"idle"};
  std::vector<OnPhaseTrigger *> phase_triggers_;
  std::vector<OnRepeatedFailureTrigger *> repeated_failure_triggers_;
  std::vector<OnFollowupOpenedTrigger *> followup_opened_triggers_;

  // Counts consecutive failed reconnect attempts. Reset to 0 on a clean
  // WS_CONNECTED event. When it hits kRepeatedFailureThreshold we fire the
  // on_repeated_failure trigger exactly once (until the count resets) — yaml
  // plays an audible error chime so the user knows the link is dead.
  uint32_t consecutive_failures_{0};
  bool repeated_failure_fired_{false};
  static constexpr uint32_t kRepeatedFailureThreshold = 5;
  // Don't reset the failure counter/flag the moment WS reconnects — a
  // flapping link (connect → 2 s later disconnect → 5 more fails → another
  // chime) would spam the user. Require kStableConnectionMs of unbroken
  // uptime before declaring "we're properly back" and re-arming the chime.
  static constexpr uint32_t kStableConnectionMs = 30000;

  // Scratch buffers reused on the hot path to avoid per-callback heap allocation.
  // These MUST stay separate: mono_buf_ is owned by the MIC path (on_mic_data_,
  // microphone task) and play_buf_ by the PLAYBACK path (handle_binary_,
  // websocket task). The two run on different FreeRTOS tasks with no lock around
  // the scratch, so sharing one vector let resize() realloc the backing store
  // out from under the other task during barge-in (mic open while the bridge
  // still flushes a cancelled reply's tail audio) — read as int16 samples that
  // is full-scale garbage, i.e. an intermittent loud speaker hiss.
  std::vector<int16_t> mono_buf_;  // mic task only
  std::vector<int16_t> play_buf_;  // websocket task only

  // Follow-up dialog window after a real turn ends. 0 disables — mic
  // closes immediately after each reply, like the original turn-based
  // pipeline. Currently 0 because XMOS AEC is too leaky and the mic
  // hears its own TTS tail during this window. Re-enable (e.g. 5000)
  // when AEC is tuned or we add wait_for_user on the server.
  static constexpr uint32_t kFollowupMs = 0;
  // Used when the server explicitly requests a follow-up via the
  // request_follow_up tool — overrides kFollowupMs for a single turn.
  // Longer than the default because the model asked a real question
  // and the user might pause before answering.
  static constexpr uint32_t kRequestFollowUpMs = 10000;
  // After start_session() we wait this long for the server to emit
  // phase=listening (i.e. server VAD heard speech). If nothing comes, the
  // user pressed wake/button and stayed silent — close the session so we
  // don't sit there with the mic open eating OpenAI minutes.
  static constexpr uint32_t kNoSpeechTimeoutMs = 7000;
  // Hard ceiling on how long we'll wait for the speaker chain to drain
  // (resampler ring + mixer source ring, via has_buffered_data()) after
  // PSRAM hits 0 before giving up and proceeding anyway. Should be >
  // the worst-case downstream buffer (resampler + mixer source ~150 ms,
  // plus play-out time of whatever was in flight) by a comfortable
  // margin, but short enough that a wedged speaker doesn't lock the
  // LED in `replying` forever.
  static constexpr uint32_t kSpeakerStopTimeoutMs = 3000;

  // Tracks the opcode of the in-flight WS message so we can route
  // continuation frames (op_code = 0) to the same handler.
  bool last_data_was_binary_{false};

  // Output volume multiplier in [0, 1], updated from yaml whenever
  // external_media_player.volume / mute changes. Defaults to 1.0 so a
  // stand-alone va_client (no media_player wiring) still plays audibly.
  float volume_{1.0f};

  // Ring buffer for pending TTS audio, allocated in PSRAM. The server can
  // burst the entire response in ~200 ms; we buffer here and drain into
  // speaker.play() from loop() to keep playback smooth.
  //
  // 2 MB / (24000 Hz × 2 B) ≈ 43 s of headroom. A 30 s monologue arriving
  // in ~1 s would peak at ~1.4 MB; this size gives ~40 % overhead on top.
  // PSRAM is 8 MB on the Voice PE so cost is negligible.
  uint8_t *audio_buf_{nullptr};
  static constexpr size_t kAudioBufBytes = 2 * 1024 * 1024;
  size_t audio_head_{0};  // read pos (next byte to play)
  size_t audio_tail_{0};  // write pos (next byte to fill)
  size_t audio_fill_{0};  // bytes currently queued (audio_tail_ ≥ audio_head_ when not wrapped)
  // ESP32-S3 is dual-core: handle_binary_ runs in the esp-idf
  // websocket task (background) while loop() runs in the main app task,
  // typically on the other core. Both touch audio_head_/tail_/fill_
  // and the PSRAM data they index; without sync we hit races where
  // fill_ goes inconsistent, the reader sees the new tail before the
  // memcpy is visible, or two non-atomic increments lose each other.
  // The DAC then plays whatever bytes happened to be at those PSRAM
  // offsets — audible as "speech drops into hiss" mid-utterance.
  // portMUX is the cheapest cross-core primitive: a spinlock that
  // also masks interrupts on the holding core. Critical sections
  // are tiny (a few field updates + ≤2 memcpys of at most a few KB
  // per WS frame), so contention is negligible.
  portMUX_TYPE ring_mux_ = portMUX_INITIALIZER_UNLOCKED;

#ifdef USE_VA_CLIENT_DIAGNOSTICS
  // Diagnostics are opt-in. Enable via `diagnostics: true` in the yaml
  // schema. The original "speech drops into hiss" race (PSRAM cross-core
  // sync, fixed in 5df34c2) made this code earn its keep, but in steady
  // state production it just clutters the log and steals a few cycles per
  // audio frame for counters nothing reads. Keeping it gated lets future
  // bug hunters flip one yaml flag to re-enable the full per-turn audit.
  //
  // Three measurements per turn, logged together when the deferred
  // phase=idle emit fires (i.e. when the speaker has actually drained):
  //
  //   1) WS frame inter-arrival time. If the bridge stalls and audio
  //      arrives in bursts with > kWsGapWarnMs silence between, the
  //      downstream chain may underrun and inject silence/noise.
  //   2) TTS clipping. Volume scaling is unity by design (vol ≤ 1), so
  //      this should never fire — it's a tripwire for anyone who
  //      reintroduces a >1 gain factor.
  //   3) Downstream underrun. speaker_->has_buffered_data() == false
  //      while audio_fill_ > 0 means the resampler/mixer/i2s chain ran
  //      dry while we still had PSRAM to feed it.
  //
  // Latency anchors, also per-turn:
  uint32_t turn_t_wake_{0};               // start_session() (wake-word handler)
  uint32_t turn_t_listening_{0};          // server's first phase=listening
  uint32_t turn_t_thinking_{0};           // server's phase=thinking (end-of-speech)
  uint32_t turn_t_first_audio_out_{0};    // first binary chunk arrived from server

  uint32_t last_binary_ms_{0};
  uint32_t ws_gap_count_{0};
  uint32_t ws_gap_max_ms_{0};
  uint32_t clipped_samples_{0};
  bool underrun_logged_this_turn_{false};
  // True once the first play() of the turn has been accepted by the speaker.
  // The underrun detector keys off this: before the first play() the chain is
  // legitimately dry (the turn just started), so checking has_buffered_data()
  // there only catches the startup transient, not a real mid-turn starvation.
  bool playback_started_this_turn_{false};
  // millis() when audio_fill_ first hit 0 in WaitingDrain. Lets us measure the
  // pure downstream tail (resampler+mixer+i2s) after the PSRAM ring emptied,
  // independent of how long the PSRAM ring itself took to drain.
  uint32_t drain_t_fill_zero_{0};
  static constexpr uint32_t kWsGapWarnMs = 80;  // > ~3× normal 20 ms frame
#endif
};

}  // namespace va_client
}  // namespace esphome
