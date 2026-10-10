# CLAUDE.md

Guidance for Claude Code (and humans) working in this repo.

## Project

This is a **customized fork** of
[`esphome/home-assistant-voice-pe`](https://github.com/esphome/home-assistant-voice-pe).
It is **not drop-in compatible with the stock Home Assistant voice
pipeline.** The fork replaces the standard HA `voice_assistant` ESPHome
component with a custom `va_client` component that streams PCM16 audio
over a plain WebSocket to a backend running OpenAI's Realtime API
(the `voice-assistant` Node service in the sibling repo).

In the smart-home stack the device is therefore a **thin client**:

```
Voice PE (this firmware)
  │   wake word (micro_wake_word) runs locally on the ESP32
  │   XMOS DSP does AEC / NS / IC / AGC (configured via voice_kit over I2C)
  │
  └── WebSocket /voice (bearer = this device's token)
        ──▶ voice-assistant container :3001
              ──▶ OpenAI Realtime API (gpt-realtime-2)
              ──▶ HA MCP server (tools only, no voice pipeline)
```

There is no HA `voice_assistant` integration on the audio path
anymore. STT/TTS happen inside the Realtime session in the backend;
the device just streams mic audio up and plays speaker audio down.

Both va-direct configs DO keep a native `api:` connection to HA, but only
as an **entity/announcement channel**: HA sees the device and its
`Media Player` entity and can play announcements from scripts/automations
(`tts.speak`, `media_player.play_media` with `announce: true`) through the
announcement mixer input, concurrently with the assistant's voice. On the
Voice PE this also exposes Mute, LED Ring, Restart and a hidden-by-default
`Disable physical controls` switch (upstream's button lock: ignores the center
button and the dial);
on the Atom Echo the mixer + announcement/media chains were added for this
(its shared-mutex I2S bus is arbitrated by `bus_arbiter.h` — see below; no
wake word while media plays). Both configs expose a `Wake word sensitivity`
select (per-device probability cutoffs, `restore_value: true`) — the Atom's
option set drops hey_mycroft, which it doesn't ship. Both configs declare a media_pipeline too, so
HA transcodes plain play_media (media library) to FLAC instead of shipping
the raw file. On the VPE the HA media path has its own mixer input
(`ha_media_mixing_input`) — `media_mixing_input` belongs to va_client (the
assistant's voice). Nothing ducks media during a voice turn.
`reboot_timeout: 0s` in both — the voice path must survive HA being down,
so the device never reboots on a missing API client.

## Configs

| File | Purpose |
| --- | --- |
| `home-assistant-voice.va-direct.yaml` | **Active config — this is what gets flashed.** Uses the custom `va_client` component. |
| `home-assistant-voice.yaml` | Original upstream config. Kept for reference / upstream sync. Not built. |
| `home-assistant-voice.8mb.yaml`, `home-assistant-voice.factory.yaml` | Other upstream variants — unused here. |
| `atom-echo-s3r.va-direct.yaml` | Thin-client config for the **M5Stack Atom Echo S3R** (ESP32-S3-PICO, ES8311 mono codec, no XMOS). Reuses `va_client` with `input_format: mono16`; single full-duplex 16 kHz I2S bus + resampler. No LED ring/timers/Improv. `on_boot` rewrites ES8311 reg 0x02 to a ×1 clock pre-multiplier: the esphome es8311 component writes its table's literal multiplier as the register code (1 → ×2), and the doubled codec clock corrupted the ADC on about half of all mic starts. See `docs/superpowers/specs/2026-06-29-atom-echo-s3r-firmware-design.md`. |
| `atom-echo-s3r.mic-diag.yaml` | Diagnostics wrapper for the Atom (`packages:` + `!extend` over the va-direct config, same device): logs per-0.5 s mic-stream stats (outliers, DC, RMS, peak) under `micdiag`, restarts the mic every 6 s and dumps ES8311 registers. Flash it to measure mic starts without talking; flash the va-direct config to go back. Not built in CI. |
| `voice-kit.yaml` | XMOS voice-kit component. Shared between configs. |
| `secrets.yaml` | **Gitignored.** Holds `va_url` (e.g. `ws://va.local:3001/voice`), `va_device_token` (the Voice PE's own bearer) and `va_device_token_atom` (the Atom Echo's own bearer) — each must be registered as a `voice` device for some user in the backend (see below). |

## Custom component: `esphome/components/va_client/`

| File | Role |
| --- | --- |
| `__init__.py` | ESPHome codegen + YAML schema. Configurable: `url`, `token`, `microphone`, `mic_channel`, `mic_gain` (1–32, default 1 = no change; digital gain on the uplink), `input_format` (`stereo32` default = XMOS stereo-int32; `mono16` = plain int16-mono codec like ES8311), `speaker`, `on_phase` (automation), `on_followup_opened` (automation), `on_repeated_failure` (automation). |
| `bus_arbiter.h` | **Shared-I2S-bus arbitration** for boards whose mic and speaker sit on one `i2s_audio` bus (the Atom). Pure and host-tested like `va_core.h`. Desired state, not edge triggers: yaml reports facts (`on_phase`, `begin_tone` / `finish_tone` for rtttl) and the `bus_reconcile` script (run every 100 ms and after each fact) turns `mic_wanted(media_busy, speaker_released)` into `micro_wake_word` start/stop — so a missed or duplicated callback heals on the next tick. Owns the tone follow-ups (`take_after_tone()` → open the session after the wake beep / commit the follow-up after its chime; a newer tone cancels them) and a tone timeout, so a tone whose finished callback never comes can't leave the mic off forever. Included from `va_client.h` only so the Atom yaml can hold one in `globals:`. Not used on the Voice PE (separate mic/speaker buses). |
| `va_core.h` | **Functional core** (phase 1 split): the protocol state machine, all watchdog/deadline time math and server-JSON handling. Pure — no esp-idf / ESPHome / FreeRTOS includes, so it compiles and runs on the host (see Testing & CI below). Side effects only via an `Actions` list the shell executes. |
| `va_client.h`, `va_client.cpp` | The **imperative shell**: WS transport (`esp_websocket_client`), mic/speaker audio plane, PSRAM ring, trigger firing. Marshals WS-task events onto the main loop via `defer()` — all text-frame handling runs on the main loop by design (the core is single-threaded). |
| `automation.h` | `OnPhaseTrigger : Trigger<std::string>` (fires on every phase transition with the new phase name), `OnFollowupOpenedTrigger : Trigger<>` (fires when a **chimed** follow-up window opens — yaml owns the chime + echo-decay gate, then calls `commit_followup_mic()`), and `OnRepeatedFailureTrigger : Trigger<>` (fires when the failure counter trips). |

### What `va_client.cpp` does

- **Mic stream**: with `input_format: stereo32` (default) pulls int32
  stereo frames from the microphone and drops to int16 mono on the selected
  `mic_channel`; with `input_format: mono16` forwards already-int16-mono
  frames. Either way `mic_gain` scales the result (clamped, and on the
  stereo32 path applied *before* the int16 narrowing so a quiet XMOS signal
  is amplified from full resolution). Ships PCM16 over the WS as binary
  frames. **Level is set per unit by measuring, not by ear** — the backend's
  `MIC_DUMP_DIR` writes the exact uplink to WAV; the VPE measured 41 dB SNR
  with 22 dB headroom (→ `mic_gain: 4`), the Atom Echo measured 7 dB SNR with
  2.7% clipping from too much analog gain (→ PGA and ADC volume cut 24 dB,
  `micro_wake_word`'s own `gain_factor` raised ×16 to compensate, since the
  analog knobs feed the wake path too).
- **Speaker playback**: incoming binary frames are PCM16 audio. A
  **2 MB PSRAM ring buffer** smooths jitter and lets us defer "ready"
  LED state until the buffer actually drains.
- **Phase machine**: `idle → listening → thinking → replying → idle`,
  driven by `phase` JSON messages from the server (`src/realtime/protocol.ts`
  in voice-assistant). Each transition fires `on_phase`.
- **Reconnect**: exponential backoff (1s / 2s / 5s / 10s, capped).
- **No-speech watchdog**: 7 s after entering `listening` with no audio
  flowing → tear the session down and return to `idle`. Cancelled once the
  server confirms speech.
- **Aborts are local**: `send_interrupt()` (Stop word, center-button cancel,
  both watchdogs) closes the turn on the device immediately — flush, mic off,
  `idle` — and only *tells* the bridge. It never waits for the server's
  answering `idle`: during a follow-up window the bridge is already idle and
  dedupes it, so waiting used to leave the mic streaming forever. In plain
  Idle it is a no-op (beyond dropping a pending follow-up window). A WS drop
  also discards any latched/pending follow-up window.
- **Max-listen watchdog** (`kMaxListeningMs`, 30 s): a hard ceiling re-armed
  on *every* entry to `listening` (wake, server-confirmed listening, follow-up).
  Backstops the case where the backend goes silent after confirming speech with
  the WS still open — without it the mic would stream and the LED stay in
  `listening` indefinitely. On expiry it sends `interrupt` and returns to `idle`.
- **Repeated-failure handling**: 5 consecutive failed connect/handshake
  attempts trip `on_repeated_failure` (used to play the error chime).
  After a 30 s stable connection the counter re-arms.

### Wire protocol

JSON messages (text frames) interleaved with binary PCM16:

- **server → device**: `hello` (handshake ack; `{audioOut, wakeChime}` — the
  wake-word beep is gated by `wakeChime`; the web panel is the single source
  of truth for that knob, there is no local switch), `phase` (state transition), `error`,
  `follow_up` (`{ms, chime?}` sent right before the end-of-turn `idle`
  after a spoken reply; `chime:true` = play the "your turn" chime — see the
  follow-up section).
- **device → server**: `start` (begin a turn — also barges in: the bridge
  cancels any reply still in flight on `start`), `interrupt` (abort the
  current turn back to idle — Stop wake word, center-button cancel, or the
  no-speech watchdog; NOT used for barge-in).
- **Turn ids (proto 2)**: every `start` carries a fresh `turn` (a per-boot
  counter, never 0); the bridge echoes it on `phase` / `follow_up` / `error`.
  `va_core` drops a tagged message unless it names the current turn *and*
  that turn is still open — a local abort closes it, so the bridge's
  answering idle, a reply or `follow_up` still in flight after a Stop, or a
  straggler from the turn before a barge-in can't reopen or drive anything
  (logged at DEBUG as "dropped server message for stale turn"). Untagged
  messages pass (proto-1 server, `hello`, anything before the first `start`).
  A server-driven follow-up turn (speech in the ambient window, no `start`)
  keeps the last start's id and stays accepted.
- The schema also defines app-level `ping`/`pong`, but this firmware never
  sends `ping` and ignores `pong` — dead surface on this side. Liveness is
  handled at the WS protocol level (the server sends protocol pings;
  `esp_websocket_client` auto-answers with pongs).

Defined in voice-assistant's `src/realtime/protocol.ts` — keep both
sides in lockstep when changing it.

## yaml-side glue (`home-assistant-voice.va-direct.yaml`)

- `va_client:` block wires the component to `secrets: va_url` /
  `va_device_token`, the i2s microphone (mic channel 0 — XMOS already
  outputs cleaned audio there), and the speaker output.
- **Decisions live in `va_core`, choreography in yaml.** yaml no longer
  reconstructs "what should happen" from LED globals; it asks the core
  (`va_client.h` forwards these):
  - `wake_action(is_stop)` → `Abort` / `Ignore` (stop word, with or without a
    turn to stop), `BargeIn` (reply audible), `Start`, `NotConnected`.
  - `button_action(announcing, media_playing)` → `StopAnnouncement`,
    `Cancel` (a turn is in progress — checked **before** HA media, which plays
    on its own mixer lane alongside a reply), `PauseMedia`, `Start`,
    `NotConnected`.
  - `stop_word_armed()` — reply audible, or the mic open in a follow-up
    window (the core tracks that itself; the old `followup_listening` global
    is gone). `on_phase` enables/disables the `stop` model from it.
  - `followup_commit_pending()` — the chimed follow-up is still waiting for
    yaml's commit (nothing pre-empted it during the chime).
  - `on_mic_muted()` — the master mute ends a listening window (a reply is
    left alone).
  The yaml asks in back-to-back nested `if`s (nothing runs between them, so
  it is one decision) and keeps only chimes, delays and speaker stops. Both
  are fuzzed through these calls (I7–I9).
- `on_phase:` maps the phase string to the global
  `voice_assistant_phase` so the original `control_leds` script can
  drive the LED ring without modification. The original script came
  from upstream; we just keep feeding it. It arms/disarms the `stop` model
  from `stop_word_armed()` on every phase change.
- **Volume (VPE)**: the media_player's volume and mute already reach the
  assistant's voice — they go down the speaker chains to the shared i2s leaf,
  i.e. the AIC3204's hardware volume / mute, which va_client's lane plays
  through too. `va_client.set_volume()` therefore stays at 1 (0 while
  muted); scaling the samples by the volume as well attenuated the voice
  twice. The Atom config still stacks its software scale on the ES8311
  register **on purpose** (keeps the assistant below announcements on the
  loud NS4150B amp) — don't unify the two without re-measuring loudness.
- **LED idle deferral** is C++-side, not yaml: `va_core` parks in
  `WaitingDrain` after the server's end-of-turn `idle` and only emits
  the final `idle` phase from `finish_drain_` once the PSRAM ring and
  the downstream speaker chain have drained — so the LED stays in
  `replying` while audio is still playing out.
- `on_followup_opened:` plays the follow-up chime, waits out the
  echo tail, then `commit_followup_mic()` opens the mic.
- `on_repeated_failure:` plays the error chime.

### Known TODOs

Several yaml blocks are marked `TODO va-direct` — features that were
tied to the upstream `voice_assistant` component and don't apply here:

- Group media player (multi-room).
- Music ducking (this firmware no longer plays music — voice-only).

If/when these come back, they'll need a custom path through `va_client`
or a sidecar HA integration.

Timers were **removed** from `home-assistant-voice.va-direct.yaml`
(the `timer_ringing` switch, `is_timer_active`/`first_active_timer`
globals, the Timer Ring/Tick LED effects, the `ring_timer`/repeat/
timer-query scripts, and the `timer_finished` sound) — they were dead
weight with no `get_timers()` API on `va_client`. Re-adding them means
a fresh custom path through `va_client`, not un-stubbing the old code.

## Secrets

`secrets.yaml` is gitignored. Required keys:

- `va_url` — backend WebSocket URL, e.g. `ws://va.local:3001/voice`.
- `va_device_token` — the Voice PE's own bearer token (used by
  `home-assistant-voice.va-direct.yaml`). The backend authenticates it
  **per device against the DB**, not a shared env value: on the WS handshake
  it hashes the presented token and looks up a `voice` identity. So the token
  must be **registered as a voice device** for some user — via the web panel's
  Users page (add a voice device) or
  `npm run users -- attach-voice --user <id> --token <this-token>`. There is
  no longer a `VA_DEVICE_TOKEN` env on the backend; each speaker can carry its
  own distinct token.
- `va_device_token_atom` — the Atom Echo's own bearer token (used by
  `atom-echo-s3r.va-direct.yaml`). Same story as above: a distinct token
  registered as its own `voice` device. Each speaker config references its
  own key, so CI's stub `secrets.yaml` must define both.
- `ap_password` — shared password for both configs' fallback AP
  (`"Voice PE Setup"` / `"${friendly_name} Setup"`), used together with
  `captive_portal:` to reconfigure Wi-Fi without a full reflash.
- All upstream secrets (WiFi creds, etc.) stay as in stock.

**Whenever a va-direct yaml starts referencing a new `!secret` key**, add it
to the CI stub too (`.github/workflows/build-va-direct.yml`, the "🔑 Stub
secrets" step) in the same change — otherwise `esphome compile` fails there
with "Secret '<name>' not defined" even though it compiles fine locally
against the real `secrets.yaml`.

If the token isn't a registered voice device, the backend rejects the WS
upgrade with `4401` and the failure counter trips after a few retries.

## Critical caveat: XMOS AEC isn't perfect

Measured on M3.2 hardware: ~10× speaker → mic leak survives AEC.
That makes barge-in / follow-up-turn detection imperfect; the
`kFollowupOpenDelayMs` guard (park in Idle until the reply's i2s/DAC
tail clears before opening the mic) is what keeps the follow-up window
from committing the reply's own echo as a phantom turn.

## Follow-up dialog window (server-driven)

After a **spoken** reply the device can reopen the mic so the user
continues without a wake word. **The server owns this decision**, not
the firmware: the bridge sends a `follow_up {ms, chime?}` message right
before the end-of-turn `phase=idle`, and only after a real reply. So a
silent `wait_for_user`, a barge-in interrupt, a tool-only response, and
the initial idle send no `follow_up` and never reopen the mic. The knobs
live in the voice-assistant **web panel** (Realtime page):
`realtime.followUpMs` (ambient window, default 8000, 0 disables),
`realtime.requestFollowUpMs` (explicit-question window, default 10000, 0
disables) and `realtime.followUpChime` (default off).

Two flavours, selected by the `chime` flag (the firmware only ever reads
the `ms` the server sends — it doesn't know which knob it came from):

- `chime:false` (ambient) — the after-every-reply window. Opens silently.
- `chime:true` — the model explicitly asked a question (its
  `request_follow_up` tool); fires independently of the ambient window and
  (if the admin left the chime on) yaml plays the chime via
  `on_followup_opened`, then `commit_followup_mic()` opens the mic.

Firmware side: the `follow_up` handler latches `ms`/`chime` into
`server_follow_up_ms_` (clamped to `kMaxFollowupMs`) / `server_follow_up_chime_`;
the subsequent idle routes through WaitingDrain → `finish_drain_`, which
consumes them once the reply has played out.

## Building / flashing

Stock ESPHome workflow against `home-assistant-voice.va-direct.yaml`:

```bash
esphome run home-assistant-voice.va-direct.yaml
```

The custom component under `esphome/components/va_client/` is picked
up automatically because the yaml `external_components:` block points
at it.

## Testing & CI

Phase 1 of the hardening split `va_client` into a functional core and
an imperative shell:

- **`va_core.h`** holds everything decidable without hardware: the
  state machine, deadline math, server-JSON parsing. It takes time as
  an injected `now_ms`, owns no timers, and returns side effects as an
  `Actions` list. Single-threaded by contract — the shell `defer()`s
  WS-task events onto the ESPHome main loop, so **all text-frame
  handling runs on the main loop by design**.
- **Host tests** live in `tests/host/` — `./tests/host/run.sh` compiles
  `test_va_core.cpp` with a plain host toolchain (vendored ArduinoJson,
  no esp-idf) and runs it. Known bugs are pinned by characterization
  tests and marked `KNOWN BUG` in `va_core.h`.
- **Bus arbitration** (`test_bus_arbiter.cpp`, also run by the same script):
  example tests plus a seeded random run modelling the Atom yaml (phases,
  tones whose callback may be lost, HA media, the speaker leaf's idle
  timeout, the 100 ms reconcile) checking B1 never deaf, B2 no bus
  contention, B3 a wake beep opens at most one session.
- **Invariant fuzzing**: the same script then runs `fuzz_va_core.cpp` —
  seeded random sequences of wake/stop/barge-in, WS drops and server
  messages in any order, checking after every step that (I1) the mic always
  closes on its own if nothing else happens, (I2) the mic only streams while
  the LED shows `listening`, (I3/I4) no sends on a dead WS / no reconnect
  while up, (I5) no follow-up window after a stop, (I6) a message tagged with
  a closed or older turn changes nothing. A failure prints the seed
  and an event trace; `FUZZ_FIRST_SEED=<n> FUZZ_SEEDS=1 ./tests/host/run.sh`
  reproduces it, `FUZZ_SEEDS`/`FUZZ_STEPS` scale a longer local run. New
  core behaviour should come with a new invariant here, not just an example
  test.
- **CI**: `.github/workflows/build-va-direct.yml` runs the host tests
  and `esphome compile`s both va-direct yamls
  (`home-assistant-voice.va-direct.yaml`, `atom-echo-s3r.va-direct.yaml`)
  with a stubbed `secrets.yaml`. The upstream `build.yml` only covers
  the stock factory/8mb variants, so this workflow is what actually
  gates the fork's own configs.

## Relation to upstream

Stay close to `esphome/home-assistant-voice-pe` so we can pull in
upstream fixes for the XMOS / voice-kit / LED stack. The minimal
delta is:

- The `va_client` component.
- The va-direct yaml (the original yaml is untouched).
- `secrets.yaml` keys.

Avoid editing upstream files unless you're cherry-picking from
upstream or fixing something that genuinely needs a fork-side change.

## Keep this file up to date

When the wire protocol changes (in lockstep with voice-assistant's
`src/realtime/protocol.ts`), when phase semantics shift, when the
follow-up window behaviour changes, or when a TODO comes off the
list — update this file in the same change.
