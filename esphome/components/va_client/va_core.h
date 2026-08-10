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
    SendStart,           // send {"type":"start"} on the WS
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
  };
  Type type;
  const char *phase{nullptr};  // EmitPhase only; points at a static literal
};

using Actions = std::vector<Action>;

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
  static constexpr uint32_t kProtoVersion = 1;

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
        // Raw interrupt send (original bypassed send_interrupt(): no flush,
        // no interrupt_pending_) guarded on the WS being up.
        if (this->connected_)
          out.push_back({Action::Type::SendInterrupt, nullptr});
        this->transition_(State::Idle, kPhaseIdle, now, out);
      }
    }
    // Max-listen watchdog: hard ceiling on time spent in Listening.
    if (this->listen_max_armed_ && time_reached_(now, this->listen_max_at_)) {
      this->listen_max_armed_ = false;
      if (this->state_ == State::Listening) {
        // Full send_interrupt(): tells the bridge to abort the wedged turn.
        // send_interrupt() latches interrupt_pending_ for a server idle to
        // consume, but the local transition to Idle below makes that idle
        // land with !turn_just_ended — nothing would ever consume it and the
        // stale flag would eat the NEXT turn's follow-up window (review
        // H2/E). Consume it here: the local Idle already closes the turn.
        this->send_interrupt(now, out);
        this->interrupt_pending_ = false;
        this->transition_(State::Idle, kPhaseIdle, now, out);
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
    this->reset_turn_modifiers_();
    this->cancel_followup_timers_();
    this->listen_max_armed_ = false;
    out.push_back({Action::Type::FlushAudioQueue, nullptr});
    this->send_start_(out);
    this->transition_(State::Listening, kPhaseListening, now, out);
    this->no_speech_at_ = now + kNoSpeechTimeoutMs;
    this->no_speech_armed_ = true;
    this->arm_listening_watchdog_(now);
  }

  // Stop / cancel: abort the current turn back to idle. No-op (bar a shell
  // warn log) when the WS is down — matches the original early return.
  void send_interrupt(uint32_t now, Actions &out) {
    (void) now;
    if (!this->connected_)
      return;
    out.push_back({Action::Type::SendInterrupt, nullptr});
    out.push_back({Action::Type::FlushAudioQueue, nullptr});
    this->clear_follow_up_latch_();
    this->no_speech_armed_ = false;
    this->cancel_followup_timers_();
    this->listen_max_armed_ = false;
    // The phase=idle the server is about to send must not open a follow-up
    // window — the user said "stop". Consumed on the next end-of-turn idle.
    this->interrupt_pending_ = true;
  }

  // Barge-in prologue (wake word during a reply, before the wake chime):
  // neutralise the state machine so the drain check can't fire finish_drain_
  // — and open a stray follow-up window — while the chime plays. Deliberately
  // pins state to Idle WITHOUT a transition/emit (the original wrote
  // current_state_ directly, skipping the log + phase emit + timestamp).
  void prepare_barge_in() {
    this->cancel_followup_timers_();
    this->reset_turn_modifiers_();
    this->state_ = State::Idle;
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
  // Exposed for tests / diagnostics.
  bool interrupt_pending() const { return this->interrupt_pending_; }
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
    out.push_back({Action::Type::SendStart, nullptr});
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
    this->interrupt_pending_ = false;
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

    if (this->interrupt_pending_) {
      // User barge-cancelled. Clean close, no drain wait (send_interrupt
      // already flushed the ring), no follow-up.
      this->interrupt_pending_ = false;
      this->clear_follow_up_latch_();
      this->transition_(State::Idle, kPhaseIdle, now, out);
      return;
    }

    // Real end of turn. If everything already played out, go straight to the
    // post-turn decision; otherwise park in WaitingDrain (deliberately with
    // NO phase emit — finish_drain_ fires the deferred LED-idle later) and
    // let on_tick drive finish_drain_ when the speaker chain empties.
    this->state_ = State::WaitingDrain;
    this->state_entered_ms_ = now;
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

    // A stop / barge-in that landed while the reply was still draining set
    // interrupt_pending_ after the phase=idle already consumed its chance —
    // honor it here: the user cancelled, NEVER open a follow-up window.
    if (this->interrupt_pending_) {
      this->interrupt_pending_ = false;
      this->clear_follow_up_latch_();
      this->transition_(State::Idle, kPhaseIdle, now, out);
      return;
    }

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

  bool interrupt_pending_{false};
  uint32_t server_follow_up_ms_{0};
  bool server_follow_up_chime_{false};
  // True while FollowupArmed is waiting for yaml's chime → commit handoff
  // (mic closed); false on the ambient path (mic open). Only meaningful in
  // FollowupArmed — both entry points set it.
  bool followup_chime_pending_{false};

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
