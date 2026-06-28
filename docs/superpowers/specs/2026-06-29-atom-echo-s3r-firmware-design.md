# Atom Echo S3R — thin-client voice firmware

**Date:** 2026-06-29
**Status:** Design approved, hardware not yet in hand (written blind, to be
flashed and validated on arrival).

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
