# Atom Echo S3R — thin-client voice firmware

**Date:** 2026-06-29
**Status:** Validated on hardware 2026-07-06. The blind design flashed and ran,
but several assumptions were wrong on real silicon — see "Hardware validation"
at the end for the corrections (pinout, analog mic, shared-bus mutex, volume).

## Goal

Run the same thin-client voice path that Voice PE runs — local wake word,
PCM16 streamed over WebSocket to `voice-assistant :3001`, speaker audio
streamed back — on an **M5Stack Atom Echo S3R** (ESP32-S3-PICO-1-N8R8,
ES8311 mono codec + NS4150B amp). Reuse the existing `va_client` component
unchanged in behavior for Voice PE; add the new board as a second config in
this repo.

This is **not** a fork of any external project and **not** a new repo. The
`va_client` component is already hardware-agnostic (deps: `network`,
`microphone`, `speaker` only — it does not depend on `voice_kit`/XMOS), so
the work is: a small generalization of `va_client` plus one new board yaml.

## Non-goals (YAGNI)

Minimal thin client only. Explicitly out of scope: LED ring / per-phase LED
animation, HA-managed timers, multi-room group media, music ducking,
factory-reset flow, Improv BLE provisioning, volume-button emulation. These
are all Voice-PE-specific UX that this device does not need.

## Hardware (M5Stack Atom Echo S3R)

| Item | Value |
| --- | --- |
| SoC | ESP32-S3-PICO-1-N8R8 (8MB flash, 8MB octal PSRAM) |
| Codec | ES8311 (mono, full-duplex: ADC + DAC on one I2S bus) |
| Amp | NS4150B, 8Ω 1W speaker |
| Mic | single MEMS |
| Button | GPIO 41 |
| LED | no controllable RGB LED per docs (green download-mode LED only) |

GPIO map (from M5 docs — **verify against schematic before flashing**):

| Function | GPIO |
| --- | --- |
| I2S MCLK | 18 |
| I2S BCLK | 17 |
| I2S WS / LRCLK | 11 |
| I2S DOUT (ESP → codec, playback) | 48 |
| I2S DIN (codec → ESP, capture) | 0 |
| ES8311 I2C SDA | 45 |
| ES8311 I2C SCL | 4 |
| NS4150B PA enable | 3 |
| Button | 41 |

> Doc caveat: the M5 page at `/core/Atom_EchoS3R` renders as "Atom VoiceS3R".
> SoC + codec match what we expect, so it is almost certainly the same board
> under a renamed page, but the pinout is unverified hardware.

## Audio format chain (the crux)

On Voice PE the rates are decoupled by ESPHome resamplers:

- **Mic:** i2s mic runs at **16 kHz**, 32-bit, stereo (XMOS output).
  `va_client` de-interleaves one channel and `>>16` to int16 mono, then ships
  16 kHz PCM16 up the WS. The bridge expects 16 kHz uplink.
- **Speaker:** `va_client` emits **24 kHz mono int16** into an ESPHome
  resampler chain that converts up to the 48 kHz i2s speaker. So the rate
  `va_client` emits is independent of the i2s hardware rate.

ES8311 is a **single full-duplex codec on one I2S bus**, so ADC and DAC must
share one sample rate. Resolution:

- Run the I2S bus at **16 kHz** both directions (mic needs 16 kHz for the
  uplink contract).
- Feed `va_client`'s 24 kHz speaker output through an ESPHome `resampler`
  speaker (24 kHz → 16 kHz) before the i2s speaker. Same pattern Voice PE
  already uses, just a different target rate.

## Component change: `va_client` gains an input format option

`va_client`'s mic reader (`va_client.cpp:639` `on_mic_data_`) hardcodes the
XMOS format: interleaved **stereo int32**, take one channel, `>>16`. The
ES8311 mic is **mono 16-bit**. Generalize:

- New yaml option `input_format` on `va_client`:
  - `stereo32` (**default**) — current behavior. Voice PE config unchanged.
  - `mono16` — samples are already int16 mono PCM; forward as-is, no
    de-interleave, no shift.
- Plumbing: `__init__.py` adds `CONF_INPUT_FORMAT` (`cv.enum`, default
  `stereo32`) → `set_input_format(...)`. Header gets a `bool mic_mono16_`
  (or small enum) + setter. `on_mic_data_` branches on it.

Default preserves the Voice PE path bit-for-bit; only the new board opts into
`mono16`.

## New config: `atom-echo-s3r.va-direct.yaml`

Structure (minimal):

- `esphome` / `esp32`: S3-PICO, flash 8MB, **octal PSRAM enabled** (needed by
  micro_wake_word and `va_client`'s 2 MB speaker ring buffer).
- `wifi` / `api` / `ota` / `logger`: standard, secrets-driven.
- `i2c`: ES8311 on SDA 45 / SCL 4.
- `i2s_audio`: one full-duplex bus — MCLK 18, BCLK 17, WS 11, DOUT 48, DIN 0.
- `audio_dac: es8311`.
- `microphone` (platform i2s_audio): 16 kHz, mono, 16-bit, external ADC.
- `speaker`: ESPHome `resampler` speaker (24 kHz in → 16 kHz out) in front of
  the i2s_audio speaker.
- `switch`: NS4150B PA enable on GPIO 3, turned on at boot.
- `micro_wake_word`: same models as Voice PE (okay nabu / hey jarvis + stop).
- `va_client`: `url` / `token` from `secrets.yaml`, `microphone` = the i2s mic,
  `speaker` = the resampler speaker, `mic_channel: 0`, `input_format: mono16`;
  `on_phase` + `on_repeated_failure` → audio chimes.
- `binary_sensor`: button GPIO 41 → `interrupt` (cancel current turn) and/or
  mute toggle.
- No LED, no ring, no timers, no factory reset, no Improv.

Secrets reuse the existing `secrets.yaml` contract: `va_url`,
`va_device_token` (must be registered as a voice device in the backend).

## Risks (ordered — validate top first on real hardware)

1. **ES8311 full-duplex in ESPHome.** The stock `es8311` component has
   historically been an output-only `audio_dac`; the ADC/mic capture path may
   need extra codec setup or a community component. This is the largest
   unknown and the first thing to test when the device arrives.
2. **Single-bus shared sample rate.** Handled via the speaker resampler, but
   full-duplex `i2s_audio` (mic + speaker sharing one bus) support must be
   confirmed on the installed ESPHome version.
3. **No hardware AEC.** Without XMOS, speaker output leaks into the mic. Mic
   streaming is already gated to the active turn and follow-up is disabled
   (`kFollowupMs = 0`), so impact is bounded, but the "stop" wake word may
   false-trigger during replies. Accepted limitation.
4. **Unverified pinout / no LED.** Confirm GPIO map against the M5 schematic;
   accept no per-phase visual feedback (chimes only).

## Testing (when hardware arrives)

1. `esphome compile atom-echo-s3r.va-direct.yaml` — catches component/schema
   issues without hardware. Can be run now, blind.
2. Flash over USB; watch logs for ES8311 init and `va_client` WS `hello`.
3. Say wake word → observe phase transitions (`listening → thinking →
   replying → idle`) in logs; confirm uplink audio and TTS playback.
4. Validate risk #1 specifically: confirm mic frames actually flow (log byte
   counts) — if the ADC path is silent, ES8311 needs extra setup.

## Verification that Voice PE is untouched

`esphome compile home-assistant-voice.va-direct.yaml` must still succeed and
the `va_client` `input_format` default must be `stereo32`, so the diff to the
Voice PE runtime is zero.

## Hardware validation (2026-07-06)

Flashed on the real device. The end-to-end path works (wake word → listening →
thinking → reply audio → idle), but the blind design had several wrong
assumptions. Corrections now in `atom-echo-s3r.va-direct.yaml`:

1. **Pinout (Risk #4) — the M5-docs GPIO map was wrong; the printed silkscreen
   is authoritative.** Corrected against the device: I2C SCL `GPIO4→GPIO0`,
   I2S LRCLK `GPIO11→GPIO3`, MCLK `GPIO18→GPIO11`, mic DIN (ASDOUT)
   `GPIO0→GPIO4`, amp enable `GPIO3→GPIO18`. SDA (45), BCLK (17), DOUT/DSDIN
   (48), button (41) were already correct. With the wrong SCL the I2C bus was
   dead ("SCL held low, no devices") and the ES8311 never initialised.

2. **Mic is ANALOG, not PDM (Risk #1 resolved).** `use_microphone: true` on the
   esphome es8311 sets reg14 BIT(6) = *enable PDM digital microphone*, routing
   the ADC to a PDM input this board doesn't have → silence. The analog ADC
   path is enabled unconditionally in `es8311::setup()`, so the correct setting
   is the default `use_microphone: false`.

3. **The shared I2S bus is a MUTEX, not simultaneous full-duplex (Risk #2).**
   `i2s_audio` guards the bus with `Mutex try_lock()`; mic and speaker cannot
   own it at once. `timeout: never` on the speaker made it hold the bus forever
   and starve the mic (endless "Driver failed to start"). Fix: finite speaker
   timeout + stop `micro_wake_word` on `thinking` (frees the bus for the reply)
   and restart it on `idle`. Consequence: no wake-word/stop detection during a
   reply — impossible on a single-owner bus.

4. **micro_wake_word start ordering.** mww finishes setup after `on_boot`
   fires, so an `on_boot` start no-ops. Started instead from `on_phase`
   (phase == "idle"), which also removes the need for a boot delay.

5. **Speaker `channel: left`, not `mono`** — the ES8311 DAC expects a stereo
   Philips frame; a mono frame is read at the wrong slot boundaries.

6. **Playback volume must be pinned low.** No `media_player` here to drive
   `va_client` volume (defaults to 1.0 = full scale), which overdrives the
   NS4150B + tiny speaker into audible distortion. Set to ~0.06 in `on_boot`.

7. **Gain staging.** `mic_gain: 36DB` + mww `gain_factor: 2`.

8. **Wake-word detection was slow — the VAD gate was the culprit.** With
   `vad:` enabled the VAD model consistently failed to confirm real speech on
   this mic (wake model fired but VAD blocked → multi-second lag). The wake
   model does NOT false-fire in silence here, so dropping `vad:` is safe and
   makes detection near-instant.

9. **Wake beep implemented (no LED exists to indicate listening).** Confirmed
   the board has no controllable RGB LED — only the fixed boot/download LED —
   so feedback is audible only. The beep is an `rtttl` playing through the raw
   `i2s_speaker`, sequenced around the mutex bus: on wake, stop mww → let
   echo_mic release the bus → `rtttl.play`; then in `on_finished_playback`
   stop the speaker, restore its volume (see below), restart mww, and finally
   `start_session`. Opening the mic from `on_finished_playback` (not a fixed
   delay) guarantees the beep is done and the bus is free.

10. **va_client does not start the mic** — it only registers a data callback
    (`va_client.cpp:86`) and relies on micro_wake_word keeping `echo_mic`
    capturing. So mww must be running during `listening`; the beep path
    restarts it after the beep or the listening mic streams silence.

11. **rtttl stomps the shared speaker's volume.** `rtttl.play` calls
    `i2s_speaker->set_volume(gain)` (`rtttl.cpp:108`) and never restores it, so
    the beep's gain sticks and attenuates every subsequent TTS reply. Fix:
    restore `i2s_speaker` volume to 1.0 in `on_finished_playback`; the reply
    level is then controlled solely by `va_client.set_volume` (≈0.12), and the
    beep loudness solely by the rtttl `gain` (60%).

Everything works end-to-end: wake → beep → listen → STT/LLM → spoken reply →
follow-up. Remaining knobs are pure taste: reply volume (`set_volume`), beep
volume (rtttl `gain`) and beep length (the rtttl string).

## Button + audible cues (added after bring-up)

- **Button (GPIO41) is a one-button toggle** (classic Atom Echo UX): tap in
  idle starts a turn (via the shared `start_turn` script, same beep + mic
  handoff as the wake word); tap mid-turn cancels (`send_interrupt`). GPIO41
  chatters, so it's debounced (`delayed_on_off: 20ms`) and uses `on_click`.

- **Audible cues** — the only feedback channel (no controllable LED). All go
  through one `play_cue` script that reuses the wake-beep mutex handoff (stop
  mww → tone → `on_finished_playback` restores speaker volume, restarts mww,
  and runs the queued after-action). Two globals pick the after-action:
  `pending_start_session` (wake/button start → open session) and
  `pending_followup` (chimed follow-up → `commit_followup_mic`); cue tones
  leave both false → just back to idle. `tone_playing` gates on_phase's
  idle→mww.start so the phase machine doesn't grab the bus mid-tone.
  - Wired: cancel/stop tone, connection-error blip (`on_repeated_failure`),
    "your turn" follow-up chime (`on_followup_opened`, i.e. the server's
    chimed follow-up only — the ambient silent window has no device hook).
  - NOT done (needs a `va_core` C++ trigger, no yaml hook exists): a
    "didn't catch that" tone on the no-speech watchdog, and a cue for the
    ambient (silent) follow-up window.
  - Caveat: `on_repeated_failure` can blip during a slow-WiFi boot (WS fails
    a few times before WiFi is up); gate it on uptime if that becomes a
    nuisance.
