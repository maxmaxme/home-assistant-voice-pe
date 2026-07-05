/*
 * va_client — declarations for the direct-streaming WebSocket client component.
 *
 * New in this fork of Home Assistant Voice: Preview Edition
 * (esphome/home-assistant-voice-pe): replaces the stock ESPHome
 * `voice_assistant` component with a thin client that streams PCM16 audio
 * directly to a voice-assistant backend over a WebSocket. Added 2026.
 *
 * Split into functional core / imperative shell: the protocol state machine,
 * watchdog/deadline math and server-JSON handling live in va_core.h (pure
 * C++17, host-testable — see tests/host/). This class is the imperative
 * shell: WS client, I2S mic/speaker, PSRAM audio ring, ESPHome triggers.
 * The core is only ever called from the ESPHome main loop; the WS task keeps
 * ONLY binary audio frames, and marshals all text frames and connect/
 * disconnect events onto the main loop via defer().
 *
 * Copyright (C) 2026 maxmaxme.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details. You should have received a copy of the GNU General Public
 * License along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/components/speaker/speaker.h"

#include "va_core.h"

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

// Fires when the device opens a CHIMED follow-up mic window (i.e. a
// server `follow_up {chime:true}` landed and the audio buffer has drained).
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
  void set_mic_mono16(bool m) { mic_mono16_ = m; }
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
  // Called from yaml at the very start of a barge-in (wake word during a reply),
  // right after flush_audio_queue() and before the wake chime. The cancelled
  // reply may still be in WaitingDrain; this pins the state machine to Idle and
  // kills the follow-up timers so the drain check can't fire finish_drain_
  // — and open a stray follow-up window — during the chime, before
  // start_session() reopens the mic. Mic stays closed (Idle) through the chime.
  void prepare_barge_in();
  // Called from yaml's on_followup_opened automation AFTER the chime has
  // finished announcing through the speaker (wait_until !is_announcing +
  // i2s tail). Opens the mic for the server-sent window. No-op if the device
  // is no longer armed (e.g. user already pressed wake before the chime
  // finished — the new session takes priority).
  void commit_followup_mic();

  // Whether the local wake-word beep should play. Set from the server `hello`
  // (the device has no HA/web control surface for it). Defaults on until the
  // first hello arrives. Read from the yaml wake handler.
  bool wake_sound_enabled() const { return this->core_.wake_sound_enabled(); }

  // Called from the static esp-idf event handler trampoline.
  void on_ws_event(int32_t event_id, void *event_data);

 protected:
  void connect_();
  void on_mic_data_(const std::vector<uint8_t> &samples);
  void handle_binary_(const uint8_t *data, size_t len);
  // Feed the PSRAM ring into the speaker chain (silence-prime + prebuffer
  // gates included). Early-returns while a priming gate holds — split out of
  // loop() so those returns can't skip the core tick (the pre-split
  // scheduler timers ran independently of loop early-returns).
  void drain_audio_ring_();
  // Run one core event on the main loop: hands the core a fresh action list
  // and executes whatever it returns. `f` receives (VaCore&, Actions&).
  template<typename F> void run_core_(F &&f);
  void execute_actions_(const Actions &actions);
  // Snapshot of the PSRAM ring fill, taken under ring_mux_ (the WS task
  // mutates fill on the other core).
  size_t audio_fill_snapshot_();
  // Send a small JSON text frame ({"type":"start"} / {"type":"interrupt"}).
  // No-op if the WS isn't connected.
  void send_text_frame_(const char *json, size_t len);
  // Fire the on_phase trigger from the main loop. ESPHome triggers fired
  // synchronously from inside a yaml-invoked lambda could re-enter the
  // calling automation, so the actual trigger fire is marshalled onto the
  // next loop iteration via defer() (as it always was).
  void emit_phase_(const std::string &phase);

  std::string url_;
  std::string token_;
  // Lifetime-stable storage referenced by esp_websocket_client_config_t.headers.
  std::string auth_header_;
  uint8_t mic_channel_{0};
  bool mic_mono16_{false};

  microphone::Microphone *mic_{nullptr};
  speaker::Speaker *speaker_{nullptr};

  // esp_websocket_client_handle_t kept opaque to avoid leaking esp-idf into the header.
  void *ws_handle_{nullptr};
  bool ws_connected_{false};

  // The pure control-plane core: state machine, watchdogs, JSON handling.
  // Single-threaded by contract — touched from the main loop ONLY (the mic
  // task reads is_mic_streaming(), a single-byte enum load, the same benign
  // cross-task read the pre-split current_state_ had).
  VaCore core_;

  std::vector<OnPhaseTrigger *> phase_triggers_;
  std::vector<OnRepeatedFailureTrigger *> repeated_failure_triggers_;
  std::vector<OnFollowupOpenedTrigger *> followup_opened_triggers_;

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

  static constexpr uint32_t kPlaybackSampleRate = 24000;  // incoming TTS PCM rate

  // Playback jitter buffer ("prebuffer"). Before starting/resuming playback we
  // hold audio in the PSRAM ring until at least this many ms have accumulated
  // (or a short deadline elapses), so the downstream resampler/mixer/i2s chain
  // starts with a cushion and a network jitter gap (we see 100-340 ms gaps)
  // doesn't dry it out → audible crackle. Re-armed whenever the ring drains to
  // empty (reply start AND post-underflow). 0 would disable it.
  static constexpr uint32_t kPlaybackPrebufferMs = 150;
  // ms→bytes at the incoming PCM rate (mono 16-bit), folded at compile time so
  // the loop() priming gate doesn't redo the multiply every tick.
  static constexpr size_t kPlaybackPrebufferBytes =
      (size_t) kPlaybackPrebufferMs * (kPlaybackSampleRate / 1000) * 2;
  // True while we're accumulating the prebuffer cushion (holding playback).
  // Touched by handle_binary_ (WS task, arms it) + loop() (main task, releases);
  // plain flag, the tiny cross-task race is harmless.
  bool playback_priming_{false};
  // millis() when priming started (first byte after the ring was empty); used
  // for the prime deadline so real-time (non-burst) audio still starts promptly.
  uint32_t prime_started_ms_{0};

  // Resampler cold-start SILENCE-PRIME (crackle fix). The resampler does NOT
  // idle-timeout: resample(stop_gracefully=false) never returns FINISHED and
  // its output mixer-source is timeout:never, so the chain stays WARM between
  // normal replies. It goes COLD only after an explicit `speaker.stop:
  // media_resampling_speaker` (yaml interrupt / "stop" / wake / follow-up),
  // which tears the task down (is_stopped()==true). The next reply then cold-
  // starts a fresh AudioResampler whose windowed-sinc FIR begins from a zero
  // state → a startup-transient click. A PSRAM prebuffer can't fix it (the
  // transient is downstream of the ring). Fix: when cold, feed kChainPrimeMs of
  // SILENCE first so the FIR settles to a clean zero output before real audio.
  // Cold = resampler is_stopped() (precise, true exactly post-speaker.stop) OR,
  // as a backup, nothing fed for > kChainColdMs. Both are only ever true at a
  // real cold reply-start, never mid-speech; a needless prime on a warm chain
  // is harmless (60 ms silence).
  static constexpr uint32_t kChainPrimeMs = 60;   // silence burst to warm the filter
  static constexpr uint32_t kChainColdMs = 600;   // backup timer; is_stopped() is the primary signal
  // The prime burst in bytes (mono 16-bit @ kPlaybackSampleRate), folded at
  // compile time.
  static constexpr size_t kChainPrimeBytes =
      (size_t) kChainPrimeMs * (kPlaybackSampleRate / 1000) * 2;
  // Bytes of silence still to feed this cold-start (24 kHz mono 16-bit). >0
  // while priming; loop() feeds silence and holds real-audio drain until 0.
  size_t chain_prime_remaining_{0};
  // millis() of the last time we fed the resampler ANYTHING (silence or real).
  // Used to detect a cold chain: now - last_fed_ms_ > kChainColdMs. 0 = never fed.
  uint32_t last_fed_ms_{0};

#ifdef USE_VA_CLIENT_DIAGNOSTICS
  // Diagnostics are opt-in. Enable via `diagnostics: true` in the yaml
  // schema. The original "speech drops into hiss" race (PSRAM cross-core
  // sync, fixed in 5df34c2) made this code earn its keep, but in steady
  // state production it just clutters the log and steals a few cycles per
  // audio frame for counters nothing reads. Keeping it gated lets future
  // bug hunters flip one yaml flag to re-enable the full per-turn audit.
  //
  // Three measurements per turn, logged together when the core reports the
  // reply fully played out (Action LogTurnStats):
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
  static constexpr uint32_t kWsGapWarnMs = 80;  // > ~3× normal 20 ms frame
#endif
};

}  // namespace va_client
}  // namespace esphome
