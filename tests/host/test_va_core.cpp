// Host tests for va_core (the pure control-plane core of the va_client
// ESPHome component). Started as characterization tests pinning the pre-split
// behavior; the phase-2 hardening pass flipped the pinned known-bug
// expectations (reviews H2/B, H2/C, H2/E) into the correct behavior and added
// coverage for hello validation (M2/M3) and the reply-audio gate (L2).

#include "harness.h"

#include "../../esphome/components/va_client/va_core.h"

#include <string>

using esphome::va_client::Action;
using esphome::va_client::Actions;
using esphome::va_client::VaCore;
using State = VaCore::State;
using AT = Action::Type;

namespace {

// Drives the core the way the shell does: injected time, per-tick audio
// fill / speaker-drained snapshots, accumulated action log.
struct Fixture {
  VaCore core;
  Actions log;
  uint32_t now = 100000;  // non-zero so 0-sentinel fields behave like on-device
  size_t fill = 0;
  bool drained = true;

  void tick(uint32_t advance_ms = 0) {
    now += advance_ms;
    core.on_tick(now, fill, drained, log);
  }
  // Advance in small steps, ticking like the main loop does (~10 ms).
  void run_for(uint32_t ms, uint32_t step = 10) {
    for (uint32_t t = 0; t < ms; t += step)
      tick(step);
  }
  void text(const std::string &s) { core.on_server_text(s.c_str(), s.size(), now, fill, log); }
  void phase(const char *p) { text(std::string("{\"type\":\"phase\",\"value\":\"") + p + "\"}"); }
  void connect() { core.on_ws_connected(now, fill, log); }
  void disconnect() { core.on_ws_disconnected(now, fill, log); }
  void wake() { core.start_session(now, log); }
  void interrupt() { core.send_interrupt(now, log); }
  void commit() { core.commit_followup_mic(now, log); }

  int count(AT t) const {
    int n = 0;
    for (const auto &a : log)
      if (a.type == t)
        n++;
    return n;
  }
  int phase_count(const char *p) const {
    int n = 0;
    for (const auto &a : log)
      if (a.type == AT::EmitPhase && std::string(a.phase) == p)
        n++;
    return n;
  }
  const char *last_phase() const {
    const char *p = nullptr;
    for (const auto &a : log)
      if (a.type == AT::EmitPhase)
        p = a.phase;
    return p ? p : "<none>";
  }
  void clear() { log.clear(); }

  // Connected device, one full spoken turn up to (and including) the server's
  // end-of-turn phase=idle, with `pre_idle` injected right before the idle
  // (e.g. a follow_up message). Leaves the core in WaitingDrain with fill>0.
  void spoken_turn_until_idle(const std::string &pre_idle = "") {
    wake();
    phase("listening");
    phase("thinking");
    phase("replying");
    fill = 4096;  // TTS queued in the PSRAM ring
    drained = false;
    if (!pre_idle.empty())
      text(pre_idle);
    phase("idle");
  }
  // Let the queued audio play out and the downstream chain drain.
  void play_out() {
    run_for(500);
    fill = 0;
    run_for(100);
    drained = true;
    tick(10);
  }
};

}  // namespace

// ---- Initial state / basics -------------------------------------------------

TEST(initial_state_is_idle_disconnected) {
  Fixture f;
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK(!f.core.connected());
  CHECK(!f.core.is_mic_streaming());
  CHECK(f.core.wake_sound_enabled());
  CHECK_EQ(std::string(f.core.current_phase()), "idle");
}

TEST(connect_parks_in_idle_and_emits_phase) {
  Fixture f;
  f.connect();
  CHECK(f.core.connected());
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
}

TEST(reply_audio_accepted_only_in_reply_states) {
  // Review L2: binary frames must be dropped unless the state expects reply
  // audio (Thinking covers first chunks racing the deferred phase=replying;
  // WaitingDrain covers the tail).
  Fixture f;
  f.connect();
  CHECK(!f.core.accepts_reply_audio());  // Idle
  f.wake();
  CHECK(!f.core.accepts_reply_audio());  // Listening
  f.phase("thinking");
  CHECK(f.core.accepts_reply_audio());
  f.phase("replying");
  CHECK(f.core.accepts_reply_audio());
  f.fill = 1000;
  f.drained = false;
  f.text("{\"type\":\"follow_up\",\"ms\":8000,\"chime\":true}");
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::WaitingDrain);
  CHECK(f.core.accepts_reply_audio());
  f.play_out();
  CHECK_EQ(f.core.state(), State::FollowupArmed);
  CHECK(!f.core.accepts_reply_audio());
}

TEST(mic_gating_per_state) {
  Fixture f;
  f.connect();
  CHECK(!f.core.is_mic_streaming());  // Idle
  f.wake();
  CHECK(f.core.is_mic_streaming());  // Listening
  f.phase("thinking");
  CHECK(!f.core.is_mic_streaming());
  f.phase("replying");
  CHECK(!f.core.is_mic_streaming());
  f.fill = 1000;
  f.drained = false;
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::WaitingDrain);
  CHECK(!f.core.is_mic_streaming());
}

// ---- Happy path turn ---------------------------------------------------------

TEST(happy_path_full_turn) {
  Fixture f;
  f.connect();
  f.clear();

  f.wake();
  CHECK_EQ(f.count(AT::FlushAudioQueue), 1);
  CHECK_EQ(f.count(AT::SendStart), 1);
  CHECK_EQ(f.phase_count("listening"), 1);
  CHECK_EQ(f.core.state(), State::Listening);

  f.phase("listening");  // server VAD confirm
  CHECK_EQ(f.phase_count("listening"), 2);

  f.phase("thinking");
  CHECK_EQ(f.core.state(), State::Thinking);
  CHECK_EQ(f.phase_count("thinking"), 1);

  f.phase("replying");
  CHECK_EQ(f.core.state(), State::Replying);
  CHECK_EQ(f.phase_count("replying"), 1);

  // Server ends the turn while TTS is still queued: NO phase emit yet — the
  // LED stays in replying until the speaker chain actually drains.
  f.fill = 8192;
  f.drained = false;
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::WaitingDrain);
  CHECK_EQ(f.phase_count("idle"), 0);

  f.run_for(1000);  // audio still queued — nothing happens
  CHECK_EQ(f.core.state(), State::WaitingDrain);

  f.fill = 0;  // PSRAM ring empty, downstream chain still playing
  f.tick(10);
  CHECK_EQ(f.core.state(), State::WaitingDrain);

  f.drained = true;  // downstream chain reports empty
  f.tick(10);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
  CHECK_EQ(f.count(AT::LogTurnStats), 1);
  CHECK_EQ(f.count(AT::FireFollowupOpened), 0);  // no follow_up sent
}

TEST(tool_only_turn_with_no_audio_finishes_immediately) {
  Fixture f;
  f.connect();
  f.wake();
  f.phase("listening");
  f.phase("thinking");
  f.clear();
  // idle straight from Thinking with nothing queued → immediate clean close.
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
  CHECK_EQ(f.count(AT::LogTurnStats), 1);
}

TEST(start_session_while_disconnected_still_opens_mic_without_sending) {
  Fixture f;
  f.wake();
  CHECK_EQ(f.core.state(), State::Listening);
  CHECK_EQ(f.count(AT::SendStart), 0);  // send guard: WS down
  CHECK_EQ(f.count(AT::FlushAudioQueue), 1);
}

TEST(spurious_idle_outside_turn_just_syncs_led) {
  Fixture f;
  f.connect();
  f.wake();
  f.clear();
  // idle while Listening (server never confirmed a turn) — !turn_just_ended:
  // straight to Idle, no drain wait, no follow-up consideration.
  f.fill = 1234;  // even with audio queued
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
  CHECK_EQ(f.count(AT::LogTurnStats), 0);
}

// ---- Barge-in / interrupt -----------------------------------------------------

TEST(barge_in_during_reply) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":8000}");
  CHECK_EQ(f.core.state(), State::WaitingDrain);
  f.clear();

  // Wake word during the drain: yaml flushes + prepare_barge_in pins Idle
  // silently (no phase emit) so the chime can play without finish_drain_
  // opening a stray follow-up window.
  f.core.prepare_barge_in();
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.count(AT::EmitPhase), 0);
  CHECK(!f.core.is_mic_streaming());

  // Drain check must stay silent while the chime plays.
  f.fill = 0;
  f.drained = true;
  f.run_for(500);
  CHECK_EQ(f.count(AT::EmitPhase), 0);
  CHECK_EQ(f.count(AT::FireFollowupOpened), 0);

  // After the chime: start_session begins the new turn.
  f.wake();
  CHECK_EQ(f.core.state(), State::Listening);
  CHECK_EQ(f.count(AT::SendStart), 1);
  // The latched follow_up from the cancelled turn must be gone.
  CHECK_EQ(f.core.server_follow_up_ms(), 0u);
}

TEST(interrupt_during_reply_closes_without_followup) {
  Fixture f;
  f.connect();
  f.wake();
  f.phase("listening");
  f.phase("thinking");
  f.phase("replying");
  f.clear();

  f.interrupt();
  CHECK_EQ(f.count(AT::SendInterrupt), 1);
  CHECK_EQ(f.count(AT::FlushAudioQueue), 1);
  CHECK(f.core.interrupt_pending());

  // Server acks with idle — consumed by the interrupt: clean close, no
  // drain wait, no follow-up.
  f.fill = 0;
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
  CHECK(!f.core.interrupt_pending());
  CHECK_EQ(f.count(AT::FireFollowupOpened), 0);
}

TEST(interrupt_landing_during_drain_is_honored_by_finish_drain) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":8000}");
  CHECK_EQ(f.core.state(), State::WaitingDrain);

  // Stop lands while the reply tail is still draining — past the phase=idle.
  f.interrupt();
  f.clear();
  f.play_out();
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
  // Interrupt path skips the diagnostics hook and never opens a follow-up.
  CHECK_EQ(f.count(AT::LogTurnStats), 0);
  CHECK_EQ(f.count(AT::FireFollowupOpened), 0);
  f.run_for(2000);
  CHECK_EQ(f.core.state(), State::Idle);
}

TEST(interrupt_while_disconnected_is_a_noop) {
  Fixture f;
  f.wake();
  f.clear();
  f.interrupt();
  CHECK_EQ(f.count(AT::SendInterrupt), 0);
  CHECK_EQ(f.count(AT::FlushAudioQueue), 0);
  CHECK(!f.core.interrupt_pending());
}

// ---- Watchdogs -----------------------------------------------------------------

TEST(no_speech_watchdog_aborts_silent_session) {
  Fixture f;
  f.connect();
  f.wake();
  f.clear();
  f.tick(VaCore::kNoSpeechTimeoutMs - 1);
  CHECK_EQ(f.core.state(), State::Listening);
  f.tick(1);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.count(AT::SendInterrupt), 1);
  CHECK_EQ(f.phase_count("idle"), 1);
  // Raw-send path: no flush, no pending interrupt.
  CHECK_EQ(f.count(AT::FlushAudioQueue), 0);
  CHECK(!f.core.interrupt_pending());
}

TEST(no_speech_watchdog_cancelled_by_server_confirm) {
  Fixture f;
  f.connect();
  f.wake();
  f.tick(3000);
  f.phase("listening");
  f.clear();
  f.run_for(10000);
  CHECK_EQ(f.core.state(), State::Listening);
  CHECK_EQ(f.count(AT::SendInterrupt), 0);
}

TEST(no_speech_watchdog_while_disconnected_goes_idle_without_send) {
  Fixture f;
  f.wake();  // mic opens even with WS down
  f.clear();
  f.tick(VaCore::kNoSpeechTimeoutMs);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.count(AT::SendInterrupt), 0);
}

TEST(max_listen_watchdog_aborts_wedged_turn) {
  Fixture f;
  f.connect();
  f.wake();
  f.phase("listening");  // server confirmed, no-speech watchdog cancelled
  f.clear();
  f.run_for(VaCore::kMaxListeningMs - 10);
  CHECK_EQ(f.core.state(), State::Listening);
  f.tick(10);
  CHECK_EQ(f.core.state(), State::Idle);
  // Full send_interrupt(): interrupt + flush.
  CHECK_EQ(f.count(AT::SendInterrupt), 1);
  CHECK_EQ(f.count(AT::FlushAudioQueue), 1);
  CHECK_EQ(f.phase_count("idle"), 1);
}

TEST(max_listen_watchdog_consumes_interrupt_pending) {
  // The watchdog transitions to Idle locally, so the server's answering
  // phase=idle arrives with !turn_just_ended and can never consume the
  // interrupt flag — the watchdog must not leave it latched, or it eats the
  // follow-up window of the NEXT server-driven turn (review H2/E).
  Fixture f;
  f.connect();
  f.wake();
  f.phase("listening");
  f.run_for(VaCore::kMaxListeningMs + 10);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK(!f.core.interrupt_pending());

  // Server acks the interrupt with idle — spurious idle, nothing latched.
  f.phase("idle");
  CHECK(!f.core.interrupt_pending());

  // A subsequent server-driven turn: its follow-up window opens normally.
  f.phase("thinking");
  f.phase("replying");
  f.fill = 0;
  f.drained = true;
  f.clear();
  f.text("{\"type\":\"follow_up\",\"ms\":8000}");
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::Idle);  // parked for the echo guard
  f.run_for(VaCore::kFollowupOpenDelayMs + 10);
  CHECK_EQ(f.core.state(), State::FollowupArmed);
  CHECK(f.core.is_mic_streaming());
  CHECK_EQ(f.phase_count("listening"), 1);
}

// ---- Ambient (silent) follow-up window ------------------------------------------

TEST(ambient_followup_opens_after_echo_guard_and_times_out) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":8000}");
  f.play_out();
  CHECK_EQ(f.core.state(), State::Idle);  // parked for the echo guard
  CHECK(!f.core.is_mic_streaming());
  f.clear();

  // Echo guard: mic stays closed for kFollowupOpenDelayMs.
  f.tick(VaCore::kFollowupOpenDelayMs - 1);
  CHECK_EQ(f.core.state(), State::Idle);
  f.tick(1);
  CHECK_EQ(f.core.state(), State::FollowupArmed);
  CHECK(f.core.is_mic_streaming());
  CHECK_EQ(f.phase_count("listening"), 1);  // LED shows listening
  CHECK_EQ(f.count(AT::FireFollowupOpened), 0);  // silent — no chime trigger

  // User stays silent: window expires back to Idle.
  f.clear();
  f.run_for(8000);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK(!f.core.is_mic_streaming());
  CHECK_EQ(f.phase_count("idle"), 1);
}

TEST(ambient_followup_user_speaks_becomes_real_turn) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":8000}");
  f.play_out();
  f.run_for(VaCore::kFollowupOpenDelayMs + 10);
  CHECK_EQ(f.core.state(), State::FollowupArmed);

  f.phase("listening");  // server VAD heard the user
  CHECK_EQ(f.core.state(), State::Listening);
  f.clear();
  // The window timer was cancelled — 8 s later we're still in the turn.
  f.run_for(9000);
  CHECK_EQ(f.core.state(), State::Listening);
  // ... though the max-listen ceiling still applies eventually.
  f.run_for(VaCore::kMaxListeningMs);
  CHECK_EQ(f.core.state(), State::Idle);
}

TEST(followup_ms_zero_means_no_window) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":0}");
  f.play_out();
  f.clear();
  f.run_for(5000);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.count(AT::EmitPhase), 0);
}

TEST(no_followup_message_means_no_window) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle();
  f.play_out();
  f.clear();
  f.run_for(5000);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.count(AT::EmitPhase), 0);
}

TEST(followup_window_clamped_to_max) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":999999}");
  CHECK_EQ(f.core.server_follow_up_ms(), VaCore::kMaxFollowupMs);
  f.play_out();
  f.run_for(VaCore::kFollowupOpenDelayMs + 10);
  CHECK_EQ(f.core.state(), State::FollowupArmed);
  // Window expires exactly at the clamp, not at the requested 999999 ms.
  f.run_for(VaCore::kMaxFollowupMs);
  CHECK_EQ(f.core.state(), State::Idle);
}

TEST(wake_during_echo_guard_cancels_pending_window) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":8000}");
  f.play_out();
  CHECK_EQ(f.core.state(), State::Idle);
  // Fresh wake inside the 700 ms guard.
  f.tick(300);
  f.wake();
  CHECK_EQ(f.core.state(), State::Listening);
  f.phase("listening");
  f.clear();
  // The old open-delay deadline must not fire into the new turn.
  f.run_for(1000);
  CHECK_EQ(f.core.state(), State::Listening);
}

// ---- Chimed follow-up ------------------------------------------------------------

TEST(chimed_followup_fires_trigger_then_commit_opens_mic) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":10000,\"chime\":true}");
  f.clear();
  f.play_out();
  CHECK_EQ(f.core.state(), State::FollowupArmed);
  CHECK_EQ(f.count(AT::FireFollowupOpened), 1);
  CHECK_EQ(f.phase_count("idle"), 1);  // LED shows idle while the chime plays

  // On the chimed path the mic stays CLOSED until commit_followup_mic():
  // otherwise the chime's own tail gets streamed upstream (review H2/C).
  CHECK(!f.core.is_mic_streaming());
  f.tick(300);
  CHECK(!f.core.is_mic_streaming());  // still closed while the chime plays

  // yaml finished the chime → commit opens the mic for the latched window.
  f.tick(300);
  f.clear();
  f.commit();
  CHECK(f.core.is_mic_streaming());
  CHECK_EQ(f.core.state(), State::Listening);
  CHECK_EQ(f.phase_count("listening"), 1);
  CHECK_EQ(f.core.server_follow_up_ms(), 0u);  // latch consumed

  // Silent user: window expires at the server-sent 10 s.
  f.run_for(9990);
  CHECK_EQ(f.core.state(), State::Listening);
  f.run_for(20);
  CHECK_EQ(f.core.state(), State::Idle);
}

TEST(commit_followup_mic_noop_when_preempted) {
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":10000,\"chime\":true}");
  f.play_out();
  CHECK_EQ(f.core.state(), State::FollowupArmed);

  // User pressed wake before the chime finished — the new session wins.
  f.wake();
  CHECK_EQ(f.core.state(), State::Listening);
  f.clear();
  f.commit();  // late commit from yaml: must not clobber the new turn
  CHECK_EQ(f.count(AT::EmitPhase), 0);
  CHECK_EQ(f.core.state(), State::Listening);
}

TEST(commit_followup_mic_noop_from_idle) {
  Fixture f;
  f.connect();
  f.clear();
  f.commit();
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.count(AT::EmitPhase), 0);
}

// ---- Drain timestamp is per-turn (review H2/B) --------------------------------------

TEST(followup_turn_drain_measures_a_fresh_timeout) {
  // A follow-up turn enters WaitingDrain without start_session(); the drain
  // fallback must measure THIS turn's fill-zero moment, not the previous
  // turn's latched timestamp, or the 3 s fallback fires on the first tick.
  Fixture f;
  f.connect();

  // Turn 1 (wake-started): normal drain — timestamp gets latched.
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":8000}");
  f.play_out();
  f.run_for(VaCore::kFollowupOpenDelayMs + 10);
  CHECK_EQ(f.core.state(), State::FollowupArmed);

  // Turn 2 arrives through the follow-up window — no start_session().
  f.phase("listening");
  f.run_for(2000);  // user speaks; well past 3 s since turn 1's fill-zero
  f.phase("thinking");
  f.run_for(1500);
  f.phase("replying");
  f.fill = 4096;
  f.drained = false;
  f.text("{\"type\":\"follow_up\",\"ms\":8000}");
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::WaitingDrain);
  f.clear();

  // PSRAM ring empties but the downstream chain still holds ~600 ms of
  // audio: the drain must keep waiting on the downstream signal.
  f.fill = 0;
  f.tick(10);  // drained is still false
  CHECK_EQ(f.core.state(), State::WaitingDrain);
  CHECK_EQ(f.phase_count("idle"), 0);

  // Downstream reports empty → drain completes normally.
  f.tick(200);
  f.drained = true;
  f.tick(10);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
}

TEST(followup_turn_drain_fallback_fires_after_a_fresh_3s) {
  // Same setup, but the downstream signal never comes: the fallback must
  // fire kSpeakerStopTimeoutMs after THIS turn's fill hit zero.
  Fixture f;
  f.connect();
  f.spoken_turn_until_idle("{\"type\":\"follow_up\",\"ms\":8000}");
  f.play_out();
  f.run_for(VaCore::kFollowupOpenDelayMs + 10);
  f.phase("listening");
  f.run_for(4000);
  f.phase("thinking");
  f.phase("replying");
  f.fill = 4096;
  f.drained = false;
  f.phase("idle");
  f.clear();

  f.fill = 0;
  f.tick(10);  // fill-zero moment for this turn
  f.run_for(VaCore::kSpeakerStopTimeoutMs - 20);
  CHECK_EQ(f.core.state(), State::WaitingDrain);
  f.run_for(30);
  CHECK_EQ(f.core.state(), State::Idle);
}

// ---- Reconnect / repeated failure ----------------------------------------------

TEST(reconnect_backoff_schedule) {
  Fixture f;
  f.connect();
  f.tick(10);

  // Failure 1: disconnect → reconnect scheduled at +1 s.
  f.disconnect();
  f.clear();
  f.tick(999);
  CHECK_EQ(f.count(AT::ConnectWs), 0);
  f.tick(1);
  CHECK_EQ(f.count(AT::ConnectWs), 1);

  // Failures 2..5: each retry fails immediately; delays 2/5/10/10 s.
  const uint32_t expected[] = {2000, 5000, 10000, 10000};
  for (uint32_t delay : expected) {
    f.core.on_connect_failed(f.now, f.log);
    f.clear();
    f.tick(delay - 1);
    CHECK_EQ(f.count(AT::ConnectWs), 0);
    f.tick(1);
    CHECK_EQ(f.count(AT::ConnectWs), 1);
  }
}

TEST(reconnect_delay_resets_after_successful_connect) {
  Fixture f;
  f.connect();
  f.disconnect();
  f.tick(1000);  // fires ConnectWs; backoff already bumped to 2000
  f.connect();   // this connect succeeds → backoff back to 1 s
  f.tick(10);
  f.disconnect();
  f.clear();
  f.tick(1000);
  CHECK_EQ(f.count(AT::ConnectWs), 1);
}

TEST(duplicate_disconnect_events_coalesce) {
  Fixture f;
  f.connect();
  f.disconnect();  // DISCONNECTED
  f.disconnect();  // CLOSED for the same failure
  f.disconnect();  // ERROR for the same failure
  CHECK_EQ(f.core.consecutive_failures(), 1u);
  f.clear();
  f.run_for(15000);
  CHECK_EQ(f.count(AT::ConnectWs), 1);  // one pending reconnect, not three
}

TEST(repeated_failure_fires_once_at_threshold) {
  Fixture f;
  f.core.on_connect_failed(f.now, f.log);  // failure 1
  for (int i = 2; i <= 10; i++) {
    f.run_for(11000);  // let the pending reconnect fire
    f.core.on_connect_failed(f.now, f.log);  // failure i
    if (i < (int) VaCore::kRepeatedFailureThreshold) {
      CHECK_EQ(f.count(AT::FireRepeatedFailure), 0);
    } else {
      CHECK_EQ(f.count(AT::FireRepeatedFailure), 1);  // exactly once, no re-fire
    }
  }
  CHECK_EQ(f.core.consecutive_failures(), 10u);
}

TEST(stable_connection_rearms_failure_chime) {
  Fixture f;
  for (int i = 0; i < 5; i++) {
    f.core.on_connect_failed(f.now, f.log);
    f.run_for(11000);
  }
  CHECK_EQ(f.count(AT::FireRepeatedFailure), 1);
  f.clear();

  f.connect();
  // A flap before the 30 s stability window keeps the counter + fired flag.
  f.tick(10000);
  f.disconnect();
  CHECK_EQ(f.core.consecutive_failures(), 6u);
  CHECK_EQ(f.count(AT::FireRepeatedFailure), 0);  // fired flag still latched

  // Now hold the connection for the full stability window.
  f.run_for(11000);
  f.connect();
  f.run_for(VaCore::kStableConnectionMs + 10);
  CHECK_EQ(f.core.consecutive_failures(), 0u);

  // Chime re-armed: five fresh failures fire it again.
  f.clear();
  f.disconnect();
  for (int i = 0; i < 4; i++) {
    f.run_for(11000);
    f.core.on_connect_failed(f.now, f.log);
  }
  CHECK_EQ(f.count(AT::FireRepeatedFailure), 1);
}

TEST(disconnect_mid_turn_parks_idle_and_reconnects) {
  Fixture f;
  f.connect();
  f.wake();
  f.phase("listening");
  f.clear();
  f.disconnect();
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK(!f.core.is_mic_streaming());
  CHECK_EQ(f.phase_count("idle"), 1);
  f.tick(1000);
  CHECK_EQ(f.count(AT::ConnectWs), 1);
}

// ---- Server messages: hello / error / malformed ------------------------------------

TEST(hello_gates_wake_chime) {
  Fixture f;
  f.connect();
  CHECK(f.core.wake_sound_enabled());  // default on until first hello
  f.text("{\"type\":\"hello\",\"audioOut\":\"pcm\",\"wakeChime\":false}");
  CHECK(!f.core.wake_sound_enabled());
  f.text("{\"type\":\"hello\",\"wakeChime\":true}");
  CHECK(f.core.wake_sound_enabled());
  f.text("{\"type\":\"hello\"}");  // field absent → default on
  CHECK(f.core.wake_sound_enabled());
}

TEST(hello_with_unsupported_audio_out_closes_ws) {
  // Review M2: an audioOut format we can't play must not be streamed into
  // the speaker as garbage — the shell logs and closes the WS instead.
  Fixture f;
  f.connect();
  f.clear();
  f.text("{\"type\":\"hello\",\"audioOut\":\"opus\",\"wakeChime\":false}");
  CHECK_EQ(f.count(AT::CloseWsProtocolError), 1);
  // The bad hello is not otherwise applied.
  CHECK(f.core.wake_sound_enabled());
}

TEST(hello_with_pcm_or_absent_audio_out_is_accepted) {
  Fixture f;
  f.connect();
  f.clear();
  f.text("{\"type\":\"hello\",\"audioOut\":\"pcm\"}");
  f.text("{\"type\":\"hello\"}");
  CHECK_EQ(f.count(AT::CloseWsProtocolError), 0);
}

TEST(hello_proto_mismatch_warns_but_stays_connected) {
  // Review M3: proto is advisory — a mismatch logs loudly (WarnProtoMismatch)
  // but must NOT disconnect (backward/forward tolerance).
  Fixture f;
  f.connect();
  f.clear();
  f.text("{\"type\":\"hello\",\"audioOut\":\"pcm\",\"proto\":2,\"wakeChime\":false}");
  CHECK_EQ(f.count(AT::WarnProtoMismatch), 1);
  CHECK_EQ(f.count(AT::CloseWsProtocolError), 0);
  CHECK(!f.core.wake_sound_enabled());  // hello still applied
}

TEST(hello_proto_matching_or_absent_is_silent) {
  Fixture f;
  f.connect();
  f.clear();
  f.text("{\"type\":\"hello\",\"proto\":1}");
  f.text("{\"type\":\"hello\"}");
  CHECK_EQ(f.count(AT::WarnProtoMismatch), 0);
}

TEST(server_error_plays_chime_and_goes_idle) {
  Fixture f;
  f.connect();
  f.wake();
  f.phase("listening");
  f.phase("thinking");
  f.clear();
  f.text("{\"type\":\"error\",\"message\":\"openai exploded\"}");
  CHECK_EQ(f.count(AT::FireRepeatedFailure), 1);
  CHECK_EQ(f.core.state(), State::Idle);
  CHECK_EQ(f.phase_count("idle"), 1);
  // Server-side errors do NOT count against the WS reachability counter.
  CHECK_EQ(f.core.consecutive_failures(), 0u);
}

TEST(malformed_and_unknown_messages_are_ignored) {
  Fixture f;
  f.connect();
  f.wake();
  f.phase("thinking");
  f.clear();

  f.text("{nope");                                   // malformed JSON
  f.text("[1,2,3]");                                 // wrong shape
  f.text("{\"value\":\"idle\"}");                    // missing type
  f.text("{\"type\":\"pong\"}");                     // known but unmodeled
  f.text("{\"type\":\"self_destruct\"}");            // unknown type
  f.text("{\"type\":\"phase\"}");                    // phase missing value
  f.text("{\"type\":\"phase\",\"value\":\"warp\"}"); // unknown phase
  f.text("{\"type\":\"phase\",\"value\":7}");        // non-string phase

  CHECK_EQ(f.core.state(), State::Thinking);
  CHECK((int) f.log.size() == 0);
}

// ---- follow_up latch is per-turn ---------------------------------------------------

TEST(new_turn_phases_clear_stale_followup_latch) {
  Fixture f;
  f.connect();
  f.wake();
  f.phase("listening");
  // A follow_up latched, but then the turn continues (thinking) — the latch
  // must not leak into this turn's end.
  f.text("{\"type\":\"follow_up\",\"ms\":8000}");
  f.phase("thinking");
  CHECK_EQ(f.core.server_follow_up_ms(), 0u);
  f.fill = 0;
  f.drained = true;
  f.clear();
  f.phase("idle");
  CHECK_EQ(f.core.state(), State::Idle);
  f.run_for(2000);
  CHECK_EQ(f.core.state(), State::Idle);  // no window opened
}

int main() { return run_all_tests(); }
