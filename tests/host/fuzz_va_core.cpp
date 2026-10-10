// Randomized invariant tests for va_core.
//
// The example tests in test_va_core.cpp pin known scenarios; this driver
// instead throws thousands of random-but-plausible event sequences at the
// core — wake / stop / barge-in from the yaml side, WS connects and drops,
// and server messages in ANY order (the bridge can race, dedupe or go stale,
// so the core must not rely on it being polite) — and checks invariants that
// must hold after every single step:
//
//   I1  quiescence: whatever state we are in, if nothing else happens the
//       mic closes on its own (no state may stream forever with every
//       deadline disarmed).
//   I2  the mic only streams while the LED shows `listening` (the user can
//       always see when the room is being streamed).
//   I3  start / interrupt frames are only sent on a live WS.
//   I4  no reconnect is requested while the WS is up.
//   I5  after a stop / cancel, no follow-up window opens until a new turn.
//   I6  a server message tagged with a turn the device already closed, or
//       with an older turn, changes nothing (proto-2 turn ids). The model
//       tracks turns independently of the core: +1 per start_session(), and
//       a turn is closed by any interrupt that did something.
//   I7  the "stop" word is only armed when there is something to stop — a
//       reply audible, or the mic open (in a follow-up window).
//   I8  a center-button press during a turn always cancels it (never pauses
//       HA media under a still-talking assistant).
//   I9  muting the mic closes any listening window.
//
// Besides the raw API, events are routed the way the yaml routes them —
// through wake_action() / button_action() — so the decisions are fuzzed too.
//
// Deterministic: a seeded PRNG drives everything; a failure prints the seed
// and the tail of the event trace. FUZZ_SEEDS / FUZZ_STEPS env vars scale it.

#include "../../esphome/components/va_client/va_core.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <string>

using esphome::va_client::Action;
using esphome::va_client::Actions;
using esphome::va_client::VaCore;
using AT = Action::Type;
using esphome::va_client::ButtonAction;
using esphome::va_client::WakeAction;

namespace {

// Longest time the core may legitimately keep the mic open with no further
// input: ambient open delay + max follow-up window, or a committed window
// plus the max-listen ceiling, with margin.
constexpr uint32_t kQuiescenceMs = 2 * VaCore::kMaxListeningMs + VaCore::kMaxFollowupMs;

enum class Ws : uint8_t { Connecting, Connected, Down };

struct Sim {
  std::mt19937 rng;
  VaCore core;
  Actions out;
  uint32_t now = 100000;

  // Shell model.
  Ws ws = Ws::Connecting;  // setup() starts the first connect
  size_t fill = 0;
  uint32_t drain_ticks = 0;  // ticks until the downstream chain reports drained

  // yaml model.
  bool commit_pending = false;
  uint32_t commit_at = 0;
  bool wake_pending = false;  // barge-in: start_session after the chime
  uint32_t wake_at = 0;

  // I5 bookkeeping.
  bool cancelled = false;

  // I6 model: the device's turn ids as an outside observer sees them.
  uint32_t model_turn = 0;  // 0 = no start yet
  bool model_closed = false;

  std::deque<std::string> trace;
  std::string failure;

  explicit Sim(uint32_t seed) : rng(seed) {}

  uint32_t rand(uint32_t n) { return std::uniform_int_distribution<uint32_t>(0, n - 1)(this->rng); }
  bool chance(uint32_t pct) { return this->rand(100) < pct; }

  void log(const std::string &s) {
    this->trace.push_back("t=" + std::to_string(this->now) + " " + s + " -> state=" +
                          std::to_string((int) this->core.state()) + " phase=" +
                          this->core.current_phase());
    if (this->trace.size() > 40)
      this->trace.pop_front();
  }

  void fail(const char *what) {
    if (this->failure.empty())
      this->failure = what;
  }

  // Execute the actions the core emitted, the way the shell would.
  void drain_actions() {
    for (const auto &a : this->out) {
      switch (a.type) {
        case AT::SendStart:
        case AT::SendInterrupt:
          if (this->ws != Ws::Connected || !this->core.connected())
            this->fail("I3: start/interrupt sent on a dead WS");
          break;
        case AT::ConnectWs:
          if (this->ws == Ws::Connected)
            this->fail("I4: reconnect requested while connected");
          this->ws = Ws::Connecting;
          break;
        case AT::FlushAudioQueue:
          this->fill = 0;
          break;
        case AT::FireFollowupOpened:
          if (this->cancelled)
            this->fail("I5: chimed follow-up opened after a stop");
          // yaml plays the chime, waits out the tail, then commits — or the
          // chime never finishes (10%) and the commit never comes.
          if (!this->chance(10)) {
            this->commit_pending = true;
            this->commit_at = this->now + 800 + this->rand(1500);
          }
          break;
        default:
          break;
      }
    }
    this->out.clear();
    if (this->cancelled && this->core.state() == VaCore::State::FollowupArmed)
      this->fail("I5: follow-up window opened after a stop");
  }

  void tick_once(uint32_t step) {
    this->now += step;
    if (this->fill > 0) {
      const size_t played = 480 * step;  // ~24 kHz mono PCM16
      this->fill = this->fill > played ? this->fill - played : 0;
      this->drain_ticks = 3 + this->rand(20);
    } else if (this->drain_ticks > 0) {
      this->drain_ticks--;
    }
    this->core.on_tick(this->now, this->fill, this->fill == 0 && this->drain_ticks == 0, this->out);
    this->drain_actions();
    if (this->commit_pending && static_cast<int32_t>(this->now - this->commit_at) >= 0) {
      this->commit_pending = false;
      this->core.commit_followup_mic(this->now, this->out);
      this->log("yaml commit_followup_mic");
      this->drain_actions();
    }
    if (this->wake_pending && static_cast<int32_t>(this->now - this->wake_at) >= 0) {
      this->wake_pending = false;
      this->wake();
      this->log("yaml barge-in start_session");
    }
  }

  void advance(uint32_t ms) {
    const uint32_t step = ms >= 1000 ? 50 : 10;
    for (uint32_t t = 0; t < ms; t += step)
      this->tick_once(step);
  }

  void server(const std::string &json) {
    this->core.on_server_text(json.c_str(), json.size(), this->now, this->fill, this->out);
    this->log("server " + json);
    this->drain_actions();
  }

  void wake() {
    this->cancelled = false;
    this->model_turn++;
    this->model_closed = false;
    this->core.start_session(this->now, this->out);
    this->drain_actions();
  }

  void do_interrupt(const char *label) {
    this->core.send_interrupt(this->now, this->out);
    if (!this->out.empty())
      this->model_closed = true;  // it aborted a turn (idle is a no-op)
    this->cancelled = true;
    // A chime script already running keeps going; its late commit must no-op.
    this->log(label);
    this->drain_actions();
  }

  void do_barge_in(const char *label) {
    if (this->wake_pending)
      return;
    this->fill = 0;  // yaml flushes the ring + stops the resampler
    this->core.prepare_barge_in();
    this->wake_pending = true;
    this->wake_at = this->now + 400 + this->rand(600);
    this->log(label);
  }

  // A wake-word detection routed the way the yaml does it: through
  // wake_action(), then the chosen choreography.
  void policy_wake_word() {
    const bool is_stop = this->chance(40);
    const bool armed = this->core.stop_word_armed();
    // mww only reports "stop" while the model is armed; a stray one rarely.
    if (is_stop && !armed && !this->chance(10))
      return;
    switch (this->core.wake_action(is_stop)) {
      case WakeAction::Abort:
        this->do_interrupt("yaml stop word -> abort");
        break;
      case WakeAction::BargeIn:
        this->do_barge_in("yaml wake word -> barge-in");
        break;
      case WakeAction::Start:
        this->wake();
        this->log("yaml wake word -> start");
        break;
      case WakeAction::Ignore:
      case WakeAction::NotConnected:
        this->log("yaml wake word -> nothing");
        break;
    }
  }

  void policy_button() {
    const bool announcing = this->chance(15);
    const bool media = this->chance(40);
    const ButtonAction a = this->core.button_action(announcing, media);
    if (!announcing && this->core.has_turn() && a != ButtonAction::Cancel)
      this->fail("I8: a press during a turn did not cancel it");
    switch (a) {
      case ButtonAction::Cancel:
        this->do_interrupt("yaml button -> cancel");
        break;
      case ButtonAction::Start:
        this->wake();
        this->log("yaml button -> start");
        break;
      default:
        this->log("yaml button -> media/announcement/nothing");
        break;
    }
  }

  // One random event.
  void step() {
    const uint32_t r = this->rand(100);
    if (r < 22) {
      static const uint32_t kAdv[] = {0, 10, 50, 200, 700, 1600, 5000, 9000, 31000};
      const uint32_t ms = kAdv[this->rand(sizeof(kAdv) / sizeof(kAdv[0]))];
      this->advance(ms);
      this->log("advance " + std::to_string(ms));
    } else if (r < 50) {
      if (this->ws != Ws::Connected)
        return;
      this->random_server_message();
    } else if (r < 54) {
      // Raw API, no policy in front: a start at any time. yaml refuses (error
      // chime) when the WS is down, but the flag can race the defer gap.
      if (this->ws != Ws::Connected && !this->chance(10))
        return;
      this->wake();
      this->log("raw wake");
    } else if (r < 58) {
      this->do_interrupt("raw interrupt");
    } else if (r < 60) {
      if (std::strcmp(this->core.current_phase(), "replying") == 0)
        this->do_barge_in("raw barge-in prepare");
    } else if (r < 68) {
      this->ws_event();
    } else if (r < 76) {
      // Reply audio arriving from the WS task (only when it would be kept).
      if (this->ws == Ws::Connected && this->core.accepts_reply_audio()) {
        this->fill += 2000 + this->rand(40000);
        this->log("audio in");
      }
    } else if (r < 79) {
      // A yaml commit landing at an arbitrary time (stale chime script).
      this->core.commit_followup_mic(this->now, this->out);
      this->log("stray commit_followup_mic");
      this->drain_actions();
    } else if (r < 88) {
      this->policy_wake_word();
    } else if (r < 95) {
      this->policy_button();
    } else {
      this->core.on_mic_muted(this->now, this->out);
      if (!this->out.empty())
        this->model_closed = true;
      this->log("mic muted");
      this->drain_actions();
      if (this->core.is_mic_streaming())
        this->fail("I9: mic still streaming after the mic was muted");
    }
  }

  // Pick the turn tag for a turn-scoped message: none (proto-1 server), the
  // current turn, or a stale one (the closed current turn / an older turn).
  // Returns the JSON suffix and whether the device must drop the message.
  std::string turn_tag(bool &stale) {
    stale = false;
    const uint32_t r = this->rand(100);
    if (r < 30 || this->model_turn == 0)
      return "";
    uint32_t t = this->model_turn;
    if (r >= 80 && this->model_turn > 1)
      t = this->model_turn - 1 - this->rand(this->model_turn - 1);
    stale = t != this->model_turn || this->model_closed;
    return ",\"turn\":" + std::to_string(t);
  }

  // I6: deliver a message the device must ignore and check that it did.
  void server_scoped(const std::string &head, bool stale) {
    if (!stale) {
      this->server(head + "}");
      return;
    }
    const VaCore before = this->core;
    const std::string json = head + "}";
    this->core.on_server_text(json.c_str(), json.size(), this->now, this->fill, this->out);
    this->log("server (stale) " + json);
    for (const auto &a : this->out) {
      if (a.type != AT::LogStaleDropped)
        this->fail("I6: a stale-turn message produced a side effect");
    }
    if (this->core.state() != before.state() ||
        std::strcmp(this->core.current_phase(), before.current_phase()) != 0 ||
        this->core.server_follow_up_ms() != before.server_follow_up_ms())
      this->fail("I6: a stale-turn message changed the core");
    this->drain_actions();
  }

  void random_server_message() {
    static const char *kPhases[] = {"idle", "listening", "thinking", "replying"};
    const uint32_t r = this->rand(100);
    bool stale = false;
    if (r < 70) {
      const char *p = kPhases[this->rand(4)];
      const std::string tag = this->turn_tag(stale);
      // A new (accepted) server-driven turn resets the I5 latch.
      if (!stale && (std::strcmp(p, "thinking") == 0 || std::strcmp(p, "replying") == 0))
        this->cancelled = false;
      this->server_scoped(std::string("{\"type\":\"phase\",\"value\":\"") + p + "\"" + tag,
                          stale);
    } else if (r < 85) {
      const uint32_t ms = this->chance(10) ? 999999 : this->rand(12000);
      const std::string tag = this->turn_tag(stale);
      this->server_scoped("{\"type\":\"follow_up\",\"ms\":" + std::to_string(ms) +
                              ",\"chime\":" + (this->chance(40) ? "true" : "false") + tag,
                          stale);
    } else if (r < 92) {
      const std::string tag = this->turn_tag(stale);
      this->server_scoped("{\"type\":\"error\",\"message\":\"x\"" + tag, stale);
    } else if (r < 97) {
      this->server(std::string("{\"type\":\"hello\",\"audioOut\":\"pcm\",\"wakeChime\":") +
                   (this->chance(50) ? "true" : "false") + "}");
    } else {
      this->server("{\"type\":\"phase\"");  // malformed
    }
  }

  void ws_event() {
    if (this->ws == Ws::Connecting) {
      const uint32_t r = this->rand(100);
      if (r < 60) {
        this->ws = Ws::Connected;
        this->core.on_ws_connected(this->now, this->fill, this->out);
        this->log("ws connected");
      } else if (r < 80) {
        this->ws = Ws::Down;
        this->core.on_connect_failed(this->now, this->out);
        this->log("ws connect failed");
      } else {
        this->ws = Ws::Down;
        // DISCONNECTED + CLOSED (+ ERROR) burst for one failure.
        const uint32_t n = 1 + this->rand(3);
        for (uint32_t i = 0; i < n; i++)
          this->core.on_ws_disconnected(this->now, this->fill, this->out);
        this->log("ws handshake failed x" + std::to_string(n));
      }
    } else if (this->ws == Ws::Connected && this->chance(30)) {
      this->ws = Ws::Down;
      const uint32_t n = 1 + this->rand(3);
      for (uint32_t i = 0; i < n; i++)
        this->core.on_ws_disconnected(this->now, this->fill, this->out);
      this->log("ws dropped x" + std::to_string(n));
    } else {
      return;
    }
    this->drain_actions();
  }

  // I1 on a copy, so the probe doesn't disturb the real run: no more input,
  // the speaker plays out, yaml finishes any chime — the mic must close.
  void check_quiescence() {
    VaCore probe = this->core;
    Actions sink;
    uint32_t t = this->now;
    bool commit = this->commit_pending;
    uint32_t commit_at = this->commit_at;
    for (uint32_t el = 0; el < kQuiescenceMs; el += 100) {
      t += 100;
      probe.on_tick(t, 0, true, sink);
      for (const auto &a : sink) {
        if (a.type == AT::FireFollowupOpened) {
          commit = true;
          commit_at = t + 1000;
        }
      }
      sink.clear();
      if (commit && static_cast<int32_t>(t - commit_at) >= 0) {
        commit = false;
        probe.commit_followup_mic(t, sink);
        sink.clear();
      }
    }
    if (probe.is_mic_streaming())
      this->fail("I1: mic still streaming after quiescence");
  }

  void check_invariants() {
    if (this->core.is_mic_streaming() && std::strcmp(this->core.current_phase(), "listening") != 0)
      this->fail("I2: mic streaming while LED does not show listening");
    if (this->core.stop_word_armed()) {
      if (this->core.wake_action(true) != WakeAction::Abort)
        this->fail("I7: stop word armed with nothing to stop");
      if (!this->core.reply_audible() && !this->core.is_mic_streaming())
        this->fail("I7: stop word armed while neither replying nor listening");
    }
    this->check_quiescence();
  }
};

uint32_t env_or(const char *name, uint32_t def) {
  const char *v = std::getenv(name);
  return v ? static_cast<uint32_t>(std::strtoul(v, nullptr, 10)) : def;
}

}  // namespace

int main() {
  const uint32_t seeds = env_or("FUZZ_SEEDS", 300);
  const uint32_t steps = env_or("FUZZ_STEPS", 400);
  const uint32_t first = env_or("FUZZ_FIRST_SEED", 1);
  for (uint32_t seed = first; seed < first + seeds; seed++) {
    Sim sim(seed);
    for (uint32_t i = 0; i < steps; i++) {
      sim.step();
      sim.check_invariants();
      if (!sim.failure.empty()) {
        std::printf("FAIL fuzz seed=%u step=%u: %s\n  last events:\n", seed, i, sim.failure.c_str());
        for (const auto &line : sim.trace)
          std::printf("    %s\n", line.c_str());
        std::printf("  reproduce: FUZZ_FIRST_SEED=%u FUZZ_SEEDS=1 ./tests/host/run.sh\n", seed);
        return 1;
      }
    }
  }
  std::printf("PASS fuzz: %u seeds x %u steps, invariants I1-I9 held\n", seeds, steps);
  return 0;
}
