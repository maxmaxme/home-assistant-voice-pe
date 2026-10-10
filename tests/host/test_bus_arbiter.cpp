// Host tests for bus_arbiter (shared mic/speaker I2S bus, Atom Echo S3R).
//
// Example tests pin the rules; a seeded random run then drives it the way the
// Atom yaml does — phases, tones (whose finished callback may never come),
// HA media, the speaker leaf's idle timeout, and the 100 ms reconcile that
// starts/stops micro_wake_word — and checks after every step:
//
//   B1  never deaf: once nothing needs the bus (assistant idle, no media,
//       speaker released) the mic is running again within a bounded time —
//       even if a tone's finished callback was lost.
//   B2  no contention: the mic is never left running while a tone, HA media
//       or the assistant's reply needs the bus (beyond one reconcile tick).
//   B3  a wake beep opens the session at most once, and never after a newer
//       tone (a cancel cue) replaced it.

#include "harness.h"

#include "../../esphome/components/va_client/bus_arbiter.h"

#include <cstdlib>
#include <random>
#include <string>

using esphome::va_client::AfterTone;
using esphome::va_client::BusArbiter;
using esphome::va_client::Tone;

TEST(mic_wants_the_bus_only_when_nothing_else_does) {
  BusArbiter b;
  CHECK(b.mic_wanted(false, true));    // idle: wake-word listening
  CHECK(!b.mic_wanted(true, true));    // HA media holds the speaker pipeline
  CHECK(!b.mic_wanted(false, false));  // speaker leaf not released yet
  b.on_phase("listening");
  CHECK(b.mic_wanted(false, true));  // the user's turn streams from the mic
  b.on_phase("thinking");
  CHECK(!b.mic_wanted(false, true));  // free the bus before the reply
  b.on_phase("replying");
  CHECK(!b.mic_wanted(false, true));
  b.on_phase("idle");
  b.begin_tone(Tone::Cue, 0);
  CHECK(!b.mic_wanted(false, true));
  b.finish_tone();
  CHECK(b.mic_wanted(false, true));
}

TEST(each_tone_reports_its_follow_up_once) {
  BusArbiter b;
  b.begin_tone(Tone::WakeBeep, 0);
  CHECK(b.turn_starting());
  b.finish_tone();
  b.finish_tone();  // duplicate callback: no effect
  CHECK(b.turn_starting());  // still owed until taken
  CHECK(b.take_after_tone() == AfterTone::StartSession);
  CHECK(b.take_after_tone() == AfterTone::Nothing);
  CHECK(!b.turn_starting());
  b.begin_tone(Tone::FollowupChime, 0);
  b.finish_tone();
  CHECK(b.take_after_tone() == AfterTone::CommitFollowup);
  b.begin_tone(Tone::Cue, 0);
  b.finish_tone();
  CHECK(b.take_after_tone() == AfterTone::Nothing);
}

TEST(a_cancel_cue_replaces_a_pending_wake_beep) {
  BusArbiter b;
  b.begin_tone(Tone::WakeBeep, 0);
  b.begin_tone(Tone::Cue, 100);  // button pressed again during the beep
  b.finish_tone();
  CHECK(b.take_after_tone() == AfterTone::Nothing);

  // ...also right after the beep ended, before its session was taken.
  b.begin_tone(Tone::WakeBeep, 200);
  b.finish_tone();
  b.begin_tone(Tone::Cue, 300);
  b.finish_tone();
  CHECK(b.take_after_tone() == AfterTone::Nothing);
}

TEST(a_tone_that_never_reports_finishing_times_out) {
  BusArbiter b;
  b.begin_tone(Tone::WakeBeep, 1000);
  CHECK(!b.tone_timed_out(1000 + BusArbiter::kToneTimeoutMs - 1));
  CHECK(b.tone_timed_out(1000 + BusArbiter::kToneTimeoutMs));
  b.finish_tone();
  CHECK(!b.tone_timed_out(1000 + 10 * BusArbiter::kToneTimeoutMs));
  CHECK(b.take_after_tone() == AfterTone::StartSession);
}

TEST(unknown_phase_labels_are_ignored) {
  BusArbiter b;
  b.on_phase("thinking");
  b.on_phase("bogus");
  CHECK(!b.mic_wanted(false, true));
}

// ---- Randomized run (B1–B3) -------------------------------------------------------

namespace {

struct Sim {
  std::mt19937 rng;
  BusArbiter bus;
  uint32_t now = 100000;
  std::string phase = "idle";
  bool media_busy = false;
  uint32_t speaker_release_at = 0;  // leaf holds the bus until then
  // micro_wake_word as the yaml sees it: start()/stop() take effect a tick later.
  bool mww_running = false;
  int mww_pending = 0;  // +1 start, -1 stop
  // The rtttl tone actually playing, and whether its callback will fire.
  bool tone_audible = false;
  uint32_t tone_ends_at = 0;
  bool callback_lost = false;
  // B3 bookkeeping.
  bool beep_owed = false;
  int sessions = 0;
  // B1 / B2 bookkeeping: since when the environment has been quiet, and since
  // when the mic has held the bus against a competing user.
  uint32_t quiet_since = 0;
  uint32_t contention_since = 0;
  std::string failure;

  explicit Sim(uint32_t seed) : rng(seed) {}
  uint32_t rand(uint32_t n) { return std::uniform_int_distribution<uint32_t>(0, n - 1)(this->rng); }
  bool chance(uint32_t pct) { return this->rand(100) < pct; }
  bool speaker_released() const { return static_cast<int32_t>(this->now - this->speaker_release_at) >= 0; }
  void fail(const char *w) {
    if (this->failure.empty())
      this->failure = w;
  }

  void handle_after(AfterTone a) {
    if (a == AfterTone::StartSession) {
      if (!this->beep_owed)
        this->fail("B3: a session started without a live wake beep");
      this->beep_owed = false;
      this->sessions++;
    }
  }

  // yaml's tone_done script: finish (bus released at once — speaker.stop),
  // then, after its short delay, take the follow-up. Modelled back to back.
  void tone_done() {
    this->tone_audible = false;
    this->speaker_release_at = this->now;
    this->bus.finish_tone();
    this->handle_after(this->bus.take_after_tone());
  }

  // The 100 ms interval: tone timeout, then reconcile mww with mic_wanted().
  void reconcile() {
    if (this->bus.tone_timed_out(this->now))
      this->tone_done();
    const bool want = this->bus.mic_wanted(this->media_busy, this->speaker_released());
    if (want && !this->mww_running && this->mww_pending == 0)
      this->mww_pending = +1;
    else if (!want && this->mww_running && this->mww_pending == 0)
      this->mww_pending = -1;
  }

  void tick() {
    this->now += 100;
    // mww applies the previous tick's start/stop.
    if (this->mww_pending > 0)
      this->mww_running = true;
    else if (this->mww_pending < 0)
      this->mww_running = false;
    this->mww_pending = 0;
    if (this->tone_audible && static_cast<int32_t>(this->now - this->tone_ends_at) >= 0) {
      this->tone_audible = false;
      if (!this->callback_lost)
        this->tone_done();
    }
    this->reconcile();
  }

  void begin_tone(Tone t) {
    if (t == Tone::WakeBeep)
      this->beep_owed = true;
    else
      this->beep_owed = false;  // a newer tone cancels the owed session
    this->bus.begin_tone(t, this->now);
    this->tone_audible = true;
    this->tone_ends_at = this->now + 200 + this->rand(400);
    this->callback_lost = this->chance(10);
  }

  void step() {
    const uint32_t r = this->rand(100);
    if (r < 40) {
      const uint32_t n = 1 + this->rand(30);
      for (uint32_t i = 0; i < n; i++)
        this->tick();
    } else if (r < 60) {
      static const char *kPhases[] = {"idle", "listening", "thinking", "replying"};
      this->phase = kPhases[this->rand(4)];
      this->bus.on_phase(this->phase.c_str());
      if (this->phase == "replying")  // the reply plays through the leaf
        this->speaker_release_at = this->now + 500 + this->rand(1500);
    } else if (r < 80) {
      static const Tone kTones[] = {Tone::WakeBeep, Tone::Cue, Tone::FollowupChime};
      this->begin_tone(kTones[this->rand(3)]);
    } else if (r < 90) {
      this->media_busy = !this->media_busy;
      if (!this->media_busy)  // the leaf drains its buffer after the pipeline
        this->speaker_release_at = this->now + this->rand(1000);
    } else {
      // Spurious duplicate callback from rtttl (e.g. a stop()).
      this->bus.finish_tone();
      this->handle_after(this->bus.take_after_tone());
      this->tone_audible = false;
    }
  }

  void check() {
    const bool needs_bus = this->bus.tone() != Tone::None || this->media_busy ||
                           this->phase == "thinking" || this->phase == "replying";
    // B2: one tick to issue the stop, one for mww to apply it.
    if (this->mww_running && needs_bus) {
      if (this->contention_since == 0)
        this->contention_since = this->now;
      if (this->now - this->contention_since > 200)
        this->fail("B2: mic held the bus while something else needed it");
    } else {
      this->contention_since = 0;
    }
    // B1: quiet = assistant idle, no media, speaker released, no tone audible.
    const bool quiet = (this->phase == "idle" || this->phase == "listening") && !this->media_busy &&
                       this->speaker_released() && !this->tone_audible;
    if (!quiet) {
      this->quiet_since = 0;
    } else if (this->quiet_since == 0) {
      this->quiet_since = this->now;
    } else if (this->now - this->quiet_since > BusArbiter::kToneTimeoutMs + 300 && !this->mww_running) {
      this->fail("B1: device deaf — nothing needs the bus but the mic is off");
    }
  }
};

}  // namespace

TEST(randomized_bus_arbitration_holds_b1_b2_b3) {
  const char *env = std::getenv("FUZZ_SEEDS");
  const uint32_t seeds = env ? static_cast<uint32_t>(std::strtoul(env, nullptr, 10)) : 300;
  for (uint32_t seed = 1; seed <= seeds; seed++) {
    Sim sim(seed);
    for (int i = 0; i < 400 && sim.failure.empty(); i++) {
      sim.step();
      sim.check();
    }
    if (!sim.failure.empty()) {
      std::printf("    seed=%u: %s\n", seed, sim.failure.c_str());
      CHECK(sim.failure.empty());
      return;
    }
  }
}

int main() { return run_all_tests(); }
