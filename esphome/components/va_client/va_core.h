/*
 * va_core — pure control-plane core for the va_client component.
 *
 * Functional core / imperative shell split: this header owns the protocol
 * state machine, all watchdog/deadline time math and the server-JSON
 * handling, and is deliberately free of any esp-idf / ESPHome / FreeRTOS
 * dependency so it compiles and runs on a host (see tests/host/).
 *
 * Contract:
 *   - Single-threaded: every method is called from the ESPHome main loop
 *     (the shell marshals WS-task events onto the loop via defer()).
 *   - Deterministic: time is an injected `uint32_t now_ms`; the core never
 *     reads a clock and owns no timers. The shell calls on_tick() every
 *     loop iteration and the core fires its own deadlines from there.
 *   - Side effects only via Actions: each event method appends to a caller
 *     provided Actions list which the shell executes (send WS frames, fire
 *     ESPHome triggers, flush the audio ring, reconnect the WS).
 *   - The PSRAM audio ring stays in the shell (audio plane); the core only
 *     sees its fill level as a number.
 *
 * Copyright (C) 2026 maxmaxme. GPL-3.0-or-later (see va_client.h).
 */

#pragma once

#include <ArduinoJson.h>

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>

namespace esphome {
namespace va_client {

// One side effect the shell must execute. Plain enum + a couple of POD
// fields — no std::function, no per-action allocation (the shell reuses one
// Actions vector for the component's lifetime).
struct Action {
  enum class Type : uint8_t {
    SendStart,           // send {"type":"start","turn":<turn>} on the WS
    SendInterrupt,       // send {"type":"interrupt"} on the WS
    EmitPhase,           // fire on_phase triggers with `phase` (static string)
    FireFollowupOpened,  // fire on_followup_opened triggers (chimed follow-up)
    FireRepeatedFailure, // fire on_repeated_failure triggers
    FlushAudioQueue,     // drop everything queued in the PSRAM playback ring
    ConnectWs,           // (re)start the WS client — reconnect deadline hit
    LogTurnStats,        // diagnostics hook: reply finished playing out cleanly
    CloseWsProtocolError, // log an error and close the WS — server hello
                          //   announced an audio format we can't play
    WarnProtoMismatch,   // log a loud warning — server proto version differs
    LogStaleDropped,     // diagnostics: dropped a server message tagged with
                         //   a turn we already closed (or not ours); `turn`
                         //   = the message's turn id
  };
  Type type;
  const char *phase{nullptr};  // EmitPhase only; points at a static literal
  uint32_t turn{0};            // SendStart / LogStaleDropped only
};

using Actions = std::vector<Action>;

// What a wake-word detection should do, decided from the core's own state so
// yaml never has to reconstruct it from LED globals. yaml owns only the
// choreography for each answer (chimes, delays, speaker stops).
enum class WakeAction : uint8_t {
  Ignore,        // "stop" with no turn to stop
  Abort,         // "stop" during a turn → send_interrupt()
  BargeIn,       // wake word while a reply is audible → flush, prepare, chime, start
  Start,         // wake word otherwise → chime, start_session()
  NotConnected,  // wake word with the backend down → error chime, no session
};

// What the LED ring should show for the assistant, straight from the core's
// state (yaml used to keep its own copy and overwrite it after its actions).
enum class LedPhase : uint8_t {
  Idle,
  Waiting,    // wake fired, the mic is open, no speech heard yet
  Listening,  // the server heard speech — or a follow-up window is open
  Thinking,
  Replying,   // incl. the reply tail still playing out after the server's idle
};

// What a single press of the center button should do. Inputs the core can't
// see (the HA media player) are passed in by yaml.
enum class ButtonAction : uint8_t {
  StopAnnouncement,  // an HA announcement is playing → stop it
  Cancel,            // a turn is in progress → send_interrupt()
  PauseMedia,        // only HA media is playing → pause it
  Start,             // nothing going on → start a turn
  NotConnected,      // would start, but the backend is down → error chime
};

class VaCore {
 public:
  // Mirrors the shell's original state machine one-to-one.
  enum class State : uint8_t {
    Idle,           // bridge idle, mic off, no audio queued
    Listening,      // mic streaming up; pre- and post-server-VAD confirm
    Thinking,       // server processing (incl. tool calls); mic off
    Replying,       // TTS audio coming down; mic off
    WaitingDrain,   // server said idle; waiting for the speaker chain to
                    //   actually play out before emitting LED-idle and
                    //   (maybe) opening a follow-up window
    FollowupArmed,  // a follow-up window is opening: silent ambient window
                    //   (mic already open) or the chimed handoff (yaml plays
                    //   the chime, then commit_followup_mic())
  };

  // Timing/threshold constants — same values and meanings as the pre-split
  // shell (see va_client.h history for the full rationale on each).
  static constexpr uint32_t kMaxFollowupMs = 30000;
  // Measured from the drained signal, which fires ~600 ms BEFORE physical
  // silence (has_buffered_data() sees the resampler + mixer rings, not the
  // i2s 500 ms ring or the ~100 ms DAC tail). 700 ms therefore left only
  // ~100 ms of real margin and the reply's own tail leaked through the XMOS
  // AEC into server VAD as a phantom turn. 600 tail + 800 echo decay puts the
  // ambient window on the same footing as the chimed path's 800 ms post-chime
  // delay.
  static constexpr uint32_t kFollowupOpenDelayMs = 1500;
  static constexpr uint32_t kNoSpeechTimeoutMs = 7000;
  static constexpr uint32_t kMaxListeningMs = 30000;
  static constexpr uint32_t kSpeakerStopTimeoutMs = 3000;
  static constexpr uint32_t kRepeatedFailureThreshold = 5;
  static constexpr uint32_t kStableConnectionMs = 30000;
  // Wire-protocol version this firmware speaks (see voice-assistant's
  // src/realtime/protocol.ts). A differing hello.proto only warns.
  static constexpr uint32_t kProtoVersion = 2;

  // ---- Events (imperative shell → core) ------------------------------------

  // WS opened. Resets the backoff, arms the stable-connection window and
  // parks the machine in idle (the original applied server phase "idle").
  void on_ws_connected(uint32_t now, size_t audio_fill, Actions &out) {
    this->connected_ = true;
    this->reconnect_delay_ms_ = 1000;
    // Failure counter / fired flag are NOT reset here — only after the link
    // has held for kStableConnectionMs (flap protection).
    this->stable_at_ = now + kStableConnectionMs;
    this->stable_armed_ = true;
    this->apply_server_phase_(Phase::Idle, now, audio_fill, out);
  }

  // WS dropped (DISCONNECTED / CLOSED / ERROR — the shell coalesces).
  void on_ws_disconnected(uint32_t now, size_t audio_fill, Actions &out) {
    this->connected_ = false;
    // Broke before the stability window elapsed — keep the failure counter
    // and the fired flag so a flapping link doesn't earn fresh chimes.
    this->stable_armed_ = false;
    // A follow-up window can't work without the link: drop any latched or
    // pending one so a draining reply goes straight to Idle and no ambient
    // window opens (and streams) on a dead WS.
    this->clear_follow_up_latch_();
    this->cancel_followup_timers_();
    this->apply_server_phase_(Phase::Idle, now, audio_fill, out);
    this->schedule_reconnect_(now, out);
  }

  // WS client init/start failed before any connection existed.
  void on_connect_failed(uint32_t now, Actions &out) {
    this->schedule_reconnect_(now, out);
  }

  // A complete text frame from the server. Parses + validates the JSON and
  // drives the state machine. Malformed frames / unknown types / unknown
  // phases are ignored (forward compat).
  void on_server_text(const char *data, size_t len, uint32_t now, size_t audio_fill, Actions &out) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err)
      return;

    const char *type = doc["type"].as<const char *>();
    if (type == nullptr)
      return;

    // Turn-scoped messages (proto 2) name the turn they belong to. A message
    // from a turn we already aborted locally — the bridge's answering idle, a
    // reply or follow_up still in flight when the user said "stop" — or from
    // a turn that isn't the current one is dropped whole: aborts are ours, so
    // late server traffic can't reopen or drive a closed turn.
    const bool turn_scoped = std::strcmp(type, "phase") == 0 ||
                             std::strcmp(type, "follow_up") == 0 ||
                             std::strcmp(type, "error") == 0;
    if (turn_scoped && !this->accepts_turn_(doc, out))
      return;

    if (std::strcmp(type, "error") == 0) {
      // Audible cue: reuse the repeated-failure trigger (plays the error
      // chime). consecutive_failures_ is NOT bumped — that counter is for WS
      // reachability, not server-side errors.
      out.push_back({Action::Type::FireRepeatedFailure, nullptr});
      this->apply_server_phase_(Phase::Idle, now, audio_fill, out);
      return;
    }

    if (std::strcmp(type, "follow_up") == 0) {
      // Latch the window; the upcoming phase=idle routes through
      // WaitingDrain → finish_drain_ which consumes it. Clamped so a bad
      // server config can't pin the mic open indefinitely.
      uint32_t ms = doc["ms"] | 0u;
      this->server_follow_up_ms_ = ms < kMaxFollowupMs ? ms : kMaxFollowupMs;
      this->server_follow_up_chime_ = doc["chime"] | false;
      return;
    }

    if (std::strcmp(type, "phase") == 0) {
      const char *value = doc["value"].as<const char *>();
      if (value == nullptr)
        return;
      if (std::strcmp(value, "idle") == 0) {
        this->apply_server_phase_(Phase::Idle, now, audio_fill, out);
      } else if (std::strcmp(value, "listening") == 0) {
        this->apply_server_phase_(Phase::Listening, now, audio_fill, out);
      } else if (std::strcmp(value, "thinking") == 0) {
        this->apply_server_phase_(Phase::Thinking, now, audio_fill, out);
      } else if (std::strcmp(value, "replying") == 0) {
        this->apply_server_phase_(Phase::Replying, now, audio_fill, out);
      }
      // Anything else is forward-compat noise — ignore.
      return;
    }

    if (std::strcmp(type, "hello") == 0) {
      // Garbage-audio prevention (review M2): we can only play raw PCM16 —
      // if the server announces any other output format, playing its binary
      // frames would be loud noise. Bail out before applying anything.
      const char *audio_out = doc["audioOut"].as<const char *>();
      if (!doc["audioOut"].isNull() &&
          (audio_out == nullptr || std::strcmp(audio_out, "pcm") != 0)) {
        out.push_back({Action::Type::CloseWsProtocolError, nullptr});
        return;
      }
      // Protocol version (review M3): advisory only — warn loudly on a
      // mismatch but keep the session (backward/forward tolerant). Absent
      // field (pre-versioning server) reads as 0 and stays silent.
      uint32_t proto = doc["proto"] | 0u;
      if (proto != 0 && proto != kProtoVersion) {
        out.push_back({Action::Type::WarnProtoMismatch, nullptr});
      }
      // Handshake ack: carries the admin's wake-beep preference. Default on
      // if the field is absent.
      this->wake_sound_enabled_ = doc["wakeChime"] | true;
      return;
    }
    // pong / anything we don't model yet — ignore.
  }

  // Called every main-loop iteration. `audio_fill` = bytes queued in the
  // PSRAM ring; `speaker_drained` = downstream chain reports empty. Fires
  // any deadline that has come due.
  void on_tick(uint32_t now, size_t audio_fill, bool speaker_drained, Actions &out) {
    // Reconnect backoff timer.
    if (this->reconnect_armed_ && time_reached_(now, this->reconnect_at_)) {
      this->reconnect_armed_ = false;
      out.push_back({Action::Type::ConnectWs, nullptr});
    }
    // Stable-connection window: re-arm the error chime after 30 s of
    // unbroken uptime.
    if (this->stable_armed_ && time_reached_(now, this->stable_at_)) {
      this->stable_armed_ = false;
      if (this->connected_) {
        this->consecutive_failures_ = 0;
        this->repeated_failure_fired_ = false;
      }
    }
    // No-speech watchdog: wake fired but the server never confirmed speech.
    if (this->no_speech_armed_ && time_reached_(now, this->no_speech_at_)) {
      this->no_speech_armed_ = false;
      if (this->state_ == State::Listening) {
        this->send_interrupt(now, out);
      }
    }
    // Max-listen watchdog: hard ceiling on time spent in Listening.
    if (this->listen_max_armed_ && time_reached_(now, this->listen_max_at_)) {
      this->listen_max_armed_ = false;
      if (this->state_ == State::Listening) {
        this->send_interrupt(now, out);
      }
    }
    // Silent follow-up echo-guard delay elapsed → open the ambient window.
    if (this->followup_open_armed_ && time_reached_(now, this->followup_open_at_)) {
      this->followup_open_armed_ = false;
      // Only open from a clean Idle: a wake word / Stop / new turn during
      // the guard moved us out of Idle (and cancelled this deadline anyway —
      // belt and suspenders).
      if (this->state_ == State::Idle) {
        this->followup_chime_pending_ = false;  // ambient: mic opens right away
        this->transition_(State::FollowupArmed, kPhaseListening, now, out);
        this->followup_window_at_ = now + this->pending_followup_window_ms_;
        this->followup_window_armed_ = true;
        this->followup_window_guard_ = State::FollowupArmed;
      }
    }
    // Follow-up window expiry (silent path guards on FollowupArmed, the
    // committed chimed path guards on Listening — mirrors the original two
    // set_timeout("va_followup") call sites).
    if (this->followup_window_armed_ && time_reached_(now, this->followup_window_at_)) {
      this->followup_window_armed_ = false;
      if (this->state_ == this->followup_window_guard_) {
        this->transition_(State::Idle, kPhaseIdle, now, out);
      }
    }
    // Drain bookkeeping: while in WaitingDrain with the PSRAM ring empty,
    // wait for the downstream speaker chain to report drained or for the
    // kSpeakerStopTimeoutMs fallback measured from when the ring first hit 0.
    if (this->state_ == State::WaitingDrain && audio_fill == 0) {
      if (this->drain_t_fill_zero_ == 0) {
        this->drain_t_fill_zero_ = now;
      }
      const bool timed_out = (now - this->drain_t_fill_zero_) >= kSpeakerStopTimeoutMs;
      if (speaker_drained || timed_out) {
        this->finish_drain_(now, out);
      }
    }
  }

  // ---- Commands (yaml-callable, forwarded by the shell) ---------------------

  // Wake word fired — begin a fresh turn (also barges in: `start` cancels a
  // reply still in flight server-side, FlushAudioQueue drops the local tail).
  void start_session(uint32_t now, Actions &out) {
    // A fresh turn id per start (never 0, so "no turn yet" stays distinct).
    this->turn_seq_ = this->turn_seq_ + 1 == 0 ? 1 : this->turn_seq_ + 1;
    this->current_turn_ = this->turn_seq_;
    this->turn_closed_ = false;
    this->reset_turn_modifiers_();
    this->cancel_followup_timers_();
    this->listen_max_armed_ = false;
    out.push_back({Action::Type::FlushAudioQueue, nullptr});
    this->send_start_(out);
    this->followup_turn_ = false;  // a fresh turn, even from inside a window
    this->transition_(State::Listening, kPhaseListening, now, out);
    this->no_speech_at_ = now + kNoSpeechTimeoutMs;
    this->no_speech_armed_ = true;
    this->arm_listening_watchdog_(now);
  }

  // Stop / cancel / watchdog abort: end the current turn and go Idle NOW.
  //
  // The device is authoritative for its own abort — it does not wait for the
  // server's answering phase=idle. That idle never comes when the bridge is
  // already idle (a follow-up window: the bridge's setPhase dedupes), and
  // waiting for it used to leave the mic streaming with every deadline
  // disarmed. The interrupt is still sent (WS up) so the bridge cancels the
  // in-flight response; its answering idle then lands on Idle as a spurious
  // idle (LED sync only, nothing to consume). Works with the WS down too:
  // the queued tail is flushed and the mic closed either way.
  //
  // Already Idle = no turn to abort: only drop a pending follow-up (a stop
  // during the ambient echo guard), send nothing and emit nothing — callers
  // (the Atom's stop word) rely on an idle interrupt being a no-op.
  void send_interrupt(uint32_t now, Actions &out) {
    if (this->state_ == State::Idle) {
      this->cancel_followup_timers_();
      this->clear_follow_up_latch_();
      return;
    }
    if (this->connected_)
      out.push_back({Action::Type::SendInterrupt, nullptr});
    out.push_back({Action::Type::FlushAudioQueue, nullptr});
    // Anything the server still sends for this turn is now stale.
    this->turn_closed_ = true;
    this->reset_turn_modifiers_();
    this->no_speech_armed_ = false;
    this->listen_max_armed_ = false;
    this->cancel_followup_timers_();
    this->followup_chime_pending_ = false;
    this->transition_(State::Idle, kPhaseIdle, now, out);
  }

  // Barge-in prologue (wake word during a reply, before the wake chime):
  // neutralise the state machine so the drain check can't fire finish_drain_
  // — and open a stray follow-up window — while the chime plays. Deliberately
  // pins state to Idle WITHOUT a transition/emit (the original wrote
  // current_state_ directly, skipping the log + phase emit + timestamp).
  void prepare_barge_in() {
    this->cancel_followup_timers_();
    this->reset_turn_modifiers_();
    this->followup_turn_ = false;
    this->state_ = State::Idle;
  }

  // Mic mute switched on. A listening window (wake, follow-up) would only
  // stream zeros until a watchdog fires — end it now. A reply in progress is
  // left alone: muting the mic must not silence the assistant.
  void on_mic_muted(uint32_t now, Actions &out) {
    if (this->state_ == State::Listening || this->state_ == State::FollowupArmed)
      this->send_interrupt(now, out);
  }

  // yaml's on_followup_opened automation calls this after the chime + i2s
  // tail. Opens the mic for the server-sent window. No-op if anything
  // pre-empted us out of FollowupArmed.
  void commit_followup_mic(uint32_t now, Actions &out) {
    if (this->state_ != State::FollowupArmed)
      return;
    // Fall back to kMaxFollowupMs if the latch was somehow cleared so we
    // never open an unbounded window.
    const uint32_t window_ms =
        this->server_follow_up_ms_ > 0 ? this->server_follow_up_ms_ : kMaxFollowupMs;
    this->server_follow_up_ms_ = 0;
    this->followup_chime_pending_ = false;
    this->transition_(State::Listening, kPhaseListening, now, out);
    this->followup_window_at_ = now + window_ms;
    this->followup_window_armed_ = true;
    this->followup_window_guard_ = State::Listening;
    this->arm_listening_watchdog_(now);
  }

  // ---- Input policy (pure decisions for yaml) ---------------------------------

  // A reply is still coming out of the speaker (or about to).
  bool reply_audible() const {
    return this->state_ == State::Replying || this->state_ == State::WaitingDrain;
  }

  // Some turn is in progress that a stop / cancel would end.
  bool has_turn() const { return this->state_ != State::Idle; }

  // Whether the "stop" wake-word model should be armed: while the assistant
  // is audible, or while the mic is open in a follow-up window (the user
  // changed their mind). Never during the user's own fresh utterance (a false
  // "stop" would kill it) or in idle silence.
  bool stop_word_armed() const {
    if (this->reply_audible())
      return true;
    if (!this->followup_turn_)
      return false;
    return this->state_ == State::Listening ||
           (this->state_ == State::FollowupArmed && !this->followup_chime_pending_);
  }

  WakeAction wake_action(bool is_stop) const {
    if (is_stop)
      return this->has_turn() ? WakeAction::Abort : WakeAction::Ignore;
    if (this->reply_audible())
      return WakeAction::BargeIn;
    return this->connected_ ? WakeAction::Start : WakeAction::NotConnected;
  }

  // Order matters: an announcement is the most recent sound, then the
  // assistant's own turn — it must win over HA media, which plays on its own
  // mixer lane concurrently (pausing the music while the assistant keeps
  // talking was a bug) — and only then the media.
  ButtonAction button_action(bool announcing, bool media_playing) const {
    if (announcing)
      return ButtonAction::StopAnnouncement;
    if (this->has_turn())
      return ButtonAction::Cancel;
    if (media_playing)
      return ButtonAction::PauseMedia;
    return this->connected_ ? ButtonAction::Start : ButtonAction::NotConnected;
  }

  // The ring's assistant phase. Waiting vs Listening is the no-speech
  // watchdog: armed exactly from a wake until the server confirms speech.
  LedPhase led_phase() const {
    switch (this->state_) {
      case State::Listening:
        return this->no_speech_armed_ ? LedPhase::Waiting : LedPhase::Listening;
      case State::FollowupArmed:
        return this->followup_chime_pending_ ? LedPhase::Idle : LedPhase::Listening;
      case State::Thinking:
        return LedPhase::Thinking;
      case State::Replying:
      case State::WaitingDrain:
        return LedPhase::Replying;
      case State::Idle:
      default:
        return LedPhase::Idle;
    }
  }

  // The chimed follow-up handoff is still waiting for yaml's commit (nothing
  // pre-empted the window while the chime played).
  bool followup_commit_pending() const {
    return this->state_ == State::FollowupArmed && this->followup_chime_pending_;
  }

  // ---- Queries ---------------------------------------------------------------

  // Whether the mic should forward frames to the WS. Listening covers both
  // the pre-VAD window and the active turn. FollowupArmed streams only on
  // the ambient path — on the chimed path the mic stays closed until
  // commit_followup_mic() so the chime tail isn't streamed upstream.
  bool is_mic_streaming() const {
    if (this->state_ == State::Listening)
      return true;
    return this->state_ == State::FollowupArmed && !this->followup_chime_pending_;
  }

  // Whether incoming binary WS frames are expected reply audio (review L2).
  // Thinking is included because the first audio chunks can race the
  // deferred phase=replying text frame onto the device; WaitingDrain covers
  // the reply tail still arriving after the server's end-of-turn idle.
  bool accepts_reply_audio() const {
    return this->state_ == State::Thinking || this->state_ == State::Replying ||
           this->state_ == State::WaitingDrain;
  }

  bool wake_sound_enabled() const { return this->wake_sound_enabled_; }
  State state() const { return this->state_; }
  bool connected() const { return this->connected_; }
  const char *current_phase() const { return this->current_phase_; }
  // Id of the turn the last start_session() opened (0 = none since boot).
  uint32_t current_turn() const { return this->current_turn_; }
  // Exposed for tests / diagnostics.
  uint32_t consecutive_failures() const { return this->consecutive_failures_; }
  uint32_t server_follow_up_ms() const { return this->server_follow_up_ms_; }

 private:
  enum class Phase : uint8_t { Idle, Listening, Thinking, Replying };

  static constexpr const char *kPhaseIdle = "idle";
  static constexpr const char *kPhaseListening = "listening";
  static constexpr const char *kPhaseThinking = "thinking";
  static constexpr const char *kPhaseReplying = "replying";

  // Signed-difference deadline compare so uint32 millis wraparound (~49 days)
  // behaves; equivalent to the original scheduler semantics.
  static bool time_reached_(uint32_t now, uint32_t at) {
    return static_cast<int32_t>(now - at) >= 0;
  }

  void transition_(State next, const char *phase_label, uint32_t now, Actions &out) {
    // A follow-up window opening marks the turn; it survives the commit and
    // the server's speech-heard `listening`, and ends with anything else.
    if (next == State::FollowupArmed)
      this->followup_turn_ = true;
    else if (next != State::Listening)
      this->followup_turn_ = false;
    if (this->state_ != next) {
      this->state_ = next;
      this->state_entered_ms_ = now;
    }
    this->current_phase_ = phase_label;
    out.push_back({Action::Type::EmitPhase, phase_label});
  }

  void send_start_(Actions &out) {
    if (!this->connected_)
      return;
    out.push_back({Action::Type::SendStart, nullptr, this->current_turn_});
  }

  // Untagged messages pass (proto-1 server, or nothing started yet); tagged
  // ones only for the current, still-open turn. A malformed tag is dropped.
  bool accepts_turn_(JsonDocument &doc, Actions &out) const {
    JsonVariant tag = doc["turn"];
    if (tag.isNull())
      return true;
    const uint32_t turn = tag.is<uint32_t>() ? tag.as<uint32_t>() : 0;
    if (turn != 0 && turn == this->current_turn_ && !this->turn_closed_)
      return true;
    out.push_back({Action::Type::LogStaleDropped, nullptr, turn});
    return false;
  }

  void arm_listening_watchdog_(uint32_t now) {
    // Re-armed on every entry to Listening (set_timeout replaced by name in
    // the original — arming again just refreshes the deadline).
    this->listen_max_at_ = now + kMaxListeningMs;
    this->listen_max_armed_ = true;
  }

  void cancel_followup_timers_() {
    this->followup_open_armed_ = false;
    this->followup_window_armed_ = false;
  }

  void clear_follow_up_latch_() {
    this->server_follow_up_ms_ = 0;
    this->server_follow_up_chime_ = false;
  }

  void reset_turn_modifiers_() {
    this->clear_follow_up_latch_();
    this->drain_t_fill_zero_ = 0;
  }

  void schedule_reconnect_(uint32_t now, Actions &out) {
    // Multiple WS events per failure (DISCONNECTED, CLOSED, sometimes ERROR)
    // coalesce into a single reconnect + single failure-counter bump.
    if (this->reconnect_armed_)
      return;
    this->reconnect_at_ = now + this->reconnect_delay_ms_;
    this->reconnect_armed_ = true;

    this->consecutive_failures_++;
    if (this->consecutive_failures_ >= kRepeatedFailureThreshold &&
        !this->repeated_failure_fired_) {
      this->repeated_failure_fired_ = true;
      out.push_back({Action::Type::FireRepeatedFailure, nullptr});
    }

    // Backoff schedule: 1s -> 2s -> 5s -> 10s (capped).
    if (this->reconnect_delay_ms_ < 2000) {
      this->reconnect_delay_ms_ = 2000;
    } else if (this->reconnect_delay_ms_ < 5000) {
      this->reconnect_delay_ms_ = 5000;
    } else {
      this->reconnect_delay_ms_ = 10000;
    }
  }

  void apply_server_phase_(Phase phase, uint32_t now, size_t audio_fill, Actions &out) {
    if (phase == Phase::Listening) {
      // Server heard us — the pre-speech misfire watchdog is done, but keep
      // a hard ceiling on the active turn in case the backend wedges.
      this->no_speech_armed_ = false;
      this->cancel_followup_timers_();
      this->transition_(State::Listening, kPhaseListening, now, out);
      this->arm_listening_watchdog_(now);
      return;
    }

    if (phase == Phase::Thinking || phase == Phase::Replying) {
      // A real turn in progress; cancel anything related to draining or
      // follow-up from a prior turn.
      this->cancel_followup_timers_();
      this->no_speech_armed_ = false;
      this->listen_max_armed_ = false;
      this->clear_follow_up_latch_();
      if (phase == Phase::Thinking) {
        this->transition_(State::Thinking, kPhaseThinking, now, out);
      } else {
        this->transition_(State::Replying, kPhaseReplying, now, out);
      }
      return;
    }

    // Phase::Idle — the interesting case: the state we're coming FROM
    // dictates what "idle" means.
    this->listen_max_armed_ = false;
    const State from = this->state_;
    const bool turn_just_ended = (from == State::Thinking || from == State::Replying);

    if (!turn_just_ended) {
      // Spurious idle from outside a turn (initial hello, post-disconnect).
      // Just sync the LED — no drain wait, no follow-up consideration.
      this->transition_(State::Idle, kPhaseIdle, now, out);
      return;
    }

    // Real end of turn. If everything already played out, go straight to the
    // post-turn decision; otherwise park in WaitingDrain (deliberately with
    // NO phase emit — finish_drain_ fires the deferred LED-idle later) and
    // let on_tick drive finish_drain_ when the speaker chain empties.
    this->state_ = State::WaitingDrain;
    this->state_entered_ms_ = now;
    // From Thinking (reply audio raced ahead of its `replying`, then the turn
    // ended): the tail now playing is a reply — say so, or the ring keeps
    // showing "thinking" over the audio and the stop word stays disarmed.
    // From Replying the label is already right and stays (no extra emit).
    if (from == State::Thinking && audio_fill > 0) {
      this->current_phase_ = kPhaseReplying;
      out.push_back({Action::Type::EmitPhase, kPhaseReplying});
    }
    // Fresh drain clock per turn: a follow-up turn reaches here without
    // start_session(), so the previous turn's fill-zero timestamp would make
    // the 3 s fallback fire instantly (review H2/B).
    this->drain_t_fill_zero_ = 0;
    if (audio_fill == 0) {
      this->finish_drain_(now, out);
    }
  }

  void finish_drain_(uint32_t now, Actions &out) {
    // Post-turn decision point once the PSRAM ring AND the downstream chain
    // have drained (or the timeout fallback hit): emit the deferred
    // LED-idle, then open a follow-up window or go straight to Idle.

    // Diagnostics hook — the shell logs per-turn latency/audio stats here.
    out.push_back({Action::Type::LogTurnStats, nullptr});

    if (this->server_follow_up_ms_ > 0) {
      if (this->server_follow_up_chime_) {
        // Chimed: yaml plays the chime via on_followup_opened, then calls
        // commit_followup_mic() which consumes server_follow_up_ms_ (kept
        // latched through the chime). The mic must NOT open until that
        // commit, or the chime's own tail gets streamed upstream (H2/C).
        this->server_follow_up_chime_ = false;
        this->followup_chime_pending_ = true;
        this->transition_(State::FollowupArmed, kPhaseIdle, now, out);
        out.push_back({Action::Type::FireFollowupOpened, nullptr});
        return;
      }

      // Silent (ambient) window: park in Idle for kFollowupOpenDelayMs so the
      // reply's i2s/DAC tail clears before the mic opens (XMOS AEC leak
      // guard), then FollowupArmed for the window.
      this->pending_followup_window_ms_ = this->server_follow_up_ms_;
      this->server_follow_up_ms_ = 0;
      this->transition_(State::Idle, kPhaseIdle, now, out);
      this->followup_open_at_ = now + kFollowupOpenDelayMs;
      this->followup_open_armed_ = true;
      return;
    }

    // Default path: no follow-up. Close the turn cleanly.
    this->transition_(State::Idle, kPhaseIdle, now, out);
  }

  // ---- State ---------------------------------------------------------------

  State state_{State::Idle};
  uint32_t state_entered_ms_{0};
  const char *current_phase_{kPhaseIdle};

  bool connected_{false};
  bool wake_sound_enabled_{true};

  // Turn ids (proto 2): turn_seq_ counts start_session() calls, current_turn_
  // is the open (or last) turn, turn_closed_ = we aborted it locally.
  uint32_t turn_seq_{0};
  uint32_t current_turn_{0};
  bool turn_closed_{false};

  uint32_t server_follow_up_ms_{0};
  bool server_follow_up_chime_{false};
  // True while FollowupArmed is waiting for yaml's chime → commit handoff
  // (mic closed); false on the ambient path (mic open). Only meaningful in
  // FollowupArmed — both entry points set it.
  bool followup_chime_pending_{false};
  // The current Listening / FollowupArmed came out of a follow-up window (vs
  // a fresh wake). Arms the "stop" word over the window — see
  // stop_word_armed(). Maintained by transition_().
  bool followup_turn_{false};

  // millis() when audio_fill first hit 0 in WaitingDrain. 0 = not yet
  // emptied. Reset on every WaitingDrain entry (and via reset_turn_modifiers_)
  // so the fallback always measures the current turn.
  uint32_t drain_t_fill_zero_{0};

  // Deadlines: an `armed` flag + absolute time, checked in on_tick.
  bool reconnect_armed_{false};
  uint32_t reconnect_at_{0};
  uint32_t reconnect_delay_ms_{1000};

  bool stable_armed_{false};
  uint32_t stable_at_{0};
  uint32_t consecutive_failures_{0};
  bool repeated_failure_fired_{false};

  bool no_speech_armed_{false};
  uint32_t no_speech_at_{0};

  bool listen_max_armed_{false};
  uint32_t listen_max_at_{0};

  bool followup_open_armed_{false};
  uint32_t followup_open_at_{0};
  uint32_t pending_followup_window_ms_{0};

  bool followup_window_armed_{false};
  uint32_t followup_window_at_{0};
  State followup_window_guard_{State::FollowupArmed};
};

}  // namespace va_client
}  // namespace esphome
