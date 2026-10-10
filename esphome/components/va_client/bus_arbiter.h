/*
 * bus_arbiter — who may hold a single shared I2S bus (mic vs speaker).
 *
 * For boards whose codec runs mic and speaker on ONE i2s_audio bus (the
 * M5Stack Atom Echo S3R's ES8311): ESPHome's i2s_audio treats that bus as a
 * mutex, so micro_wake_word's mic (which va_client's uplink also reads) and
 * every speaker user — the assistant's reply, rtttl tones, HA announcements
 * and media — take turns. This header decides whose turn it is.
 *
 * Desired state, not edge triggers: instead of scattering mww.start/stop over
 * phase, tone and media-player callbacks (where one missed edge leaves the
 * device deaf), the yaml reports facts here and a periodic reconcile compares
 * mic_wanted() with micro_wake_word's actual state and starts/stops it. A
 * missed or duplicated callback then heals on the next tick.
 *
 * Pure like va_core.h: no esp-idf / ESPHome includes, time is injected, so it
 * runs in the host tests (tests/host/test_bus_arbiter.cpp).
 *
 * Copyright (C) 2026 maxmaxme. GPL-3.0-or-later (see va_client.h).
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace esphome {
namespace va_client {

// A short rtttl tone that needs the bus (the mic must let go first).
enum class Tone : uint8_t {
  None,
  WakeBeep,       // start-of-turn beep → open the session when it ends
  Cue,            // confirmation / error blip → back to idle listening
  FollowupChime,  // chimed follow-up "your turn" → commit the window mic
};

// What to do once a tone is over (finished or timed out).
enum class AfterTone : uint8_t { Nothing, StartSession, CommitFollowup };

class BusArbiter {
 public:
  // No tone here is longer than ~0.5 s. Past this the tone is treated as over
  // even if rtttl never reported it (a failed start, a lost callback) — the
  // device must not stay deaf behind a tone that will never end.
  static constexpr uint32_t kToneTimeoutMs = 4000;

  // va_client's on_phase label. The reply needs the bus from `thinking` on
  // (stopping the mic there means the speaker gets it on its first try).
  void on_phase(const char *phase) {
    if (std::strcmp(phase, "idle") == 0)
      this->phase_ = Phase::Idle;
    else if (std::strcmp(phase, "listening") == 0)
      this->phase_ = Phase::Listening;
    else if (std::strcmp(phase, "thinking") == 0)
      this->phase_ = Phase::Thinking;
    else if (std::strcmp(phase, "replying") == 0)
      this->phase_ = Phase::Replying;
  }

  // A tone is about to play. A newer tone replaces an older one — including
  // a follow-up already latched by finish_tone() but not yet taken: a cancel
  // cue during (or right after) the wake beep means "don't start after all".
  void begin_tone(Tone t, uint32_t now) {
    this->tone_ = t;
    this->tone_started_ = now;
    this->after_ = AfterTone::Nothing;
  }

  // The tone is over (rtttl's on_finished_playback, or the timeout below):
  // release the bus and latch what the tone was for. Idempotent — a second
  // report (the real callback racing a timeout, a stop re-firing it) changes
  // nothing.
  void finish_tone() {
    if (this->tone_ == Tone::None)
      return;
    if (this->tone_ == Tone::WakeBeep)
      this->after_ = AfterTone::StartSession;
    else if (this->tone_ == Tone::FollowupChime)
      this->after_ = AfterTone::CommitFollowup;
    this->tone_ = Tone::None;
  }

  // What to do now that the bus is the mic's again — once.
  AfterTone take_after_tone() {
    const AfterTone a = this->after_;
    this->after_ = AfterTone::Nothing;
    return a;
  }

  // The tone has been "playing" for longer than any tone lasts.
  bool tone_timed_out(uint32_t now) const {
    return this->tone_ != Tone::None &&
           static_cast<int32_t>(now - this->tone_started_) >= static_cast<int32_t>(kToneTimeoutMs);
  }

  // Whether micro_wake_word (= the mic) should hold the bus now. Only with
  // nothing else needing it: no tone, no HA media (announcing / playing /
  // paused all hold the speaker pipeline), the speaker leaf actually released
  // (it keeps the bus for its idle timeout after playing), and the assistant
  // not about to speak.
  bool mic_wanted(bool media_busy, bool speaker_released) const {
    if (this->tone_ != Tone::None || media_busy || !speaker_released)
      return false;
    return this->phase_ == Phase::Idle || this->phase_ == Phase::Listening;
  }

  // The wake beep is playing (or just ended) and the session opens after it —
  // the turn is already under way as far as the user is concerned.
  bool turn_starting() const {
    return this->tone_ == Tone::WakeBeep || this->after_ == AfterTone::StartSession;
  }
  Tone tone() const { return this->tone_; }

 private:
  enum class Phase : uint8_t { Idle, Listening, Thinking, Replying };

  Phase phase_{Phase::Idle};
  Tone tone_{Tone::None};
  uint32_t tone_started_{0};
  AfterTone after_{AfterTone::Nothing};
};

}  // namespace va_client
}  // namespace esphome
