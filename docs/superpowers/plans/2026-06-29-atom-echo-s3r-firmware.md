# Atom Echo S3R Thin-Client Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the existing `va_client` thin-client voice path on an M5Stack Atom Echo S3R (ESP32-S3-PICO, ES8311 mono codec) via a new board config, plus a small `va_client` generalization, without changing Voice PE behavior.

**Architecture:** `va_client` gains an `input_format` option (`stereo32` default / `mono16` new) so it can read a plain mono 16-bit codec as well as the XMOS stereo-int32 stream. A new `atom-echo-s3r.va-direct.yaml` wires ES8311 (full-duplex, single 16 kHz I2S bus) to `va_client`, with an ESPHome resampler converting `va_client`'s 24 kHz speaker output down to the 16 kHz bus.

**Tech Stack:** ESPHome (esp-idf framework), ESP32-S3, ES8311 codec, `va_client` custom component (C++ / esp_websocket_client), micro_wake_word.

## Global Constraints

- ESPHome `min_version: 2026.6.0` (match Voice PE config).
- `va_client` `input_format` **default MUST be `stereo32`** — Voice PE config (`home-assistant-voice.va-direct.yaml`) must compile and behave bit-for-bit unchanged.
- Mic uplink to the backend is **16 kHz PCM16 mono** (bridge contract). `va_client` speaker output is **24 kHz mono int16** (hardcoded `va_client.cpp:100`).
- ES8311 is full-duplex on one I2S bus → ADC and DAC share one sample rate (16 kHz).
- Comments: WHY only, never WHAT. No `Co-Authored-By` trailers. No autocommit — commit only when the user explicitly asks.
- Hardware is not present; validation is `esphome config` / `esphome compile` now, on-device later. The user runs all shell commands.

---

### Task 1: Add `input_format` option to `va_client`

**Files:**
- Modify: `esphome/components/va_client/__init__.py`
- Modify: `esphome/components/va_client/va_client.h`
- Modify: `esphome/components/va_client/va_client.cpp:639-673` (`on_mic_data_`)

**Interfaces:**
- Consumes: nothing new.
- Produces: yaml key `input_format: stereo32 | mono16` on the `va_client` platform; `VaClient::set_mic_mono16(bool)` C++ setter.

- [ ] **Step 1: Add the schema option in `__init__.py`**

Add the const and schema entry, and pass it to the setter. Insert near the other `CONF_*` consts (after `CONF_MIC_CHANNEL`):

```python
CONF_INPUT_FORMAT = "input_format"

INPUT_FORMATS = {
    # XMOS path: interleaved stereo int32, one channel, real audio in high 16 bits.
    "stereo32": False,
    # Plain codec path: already int16 mono, forwarded as-is.
    "mono16": True,
}
```

Add to `CONFIG_SCHEMA` (after the `CONF_MIC_CHANNEL` line):

```python
        cv.Optional(CONF_INPUT_FORMAT, default="stereo32"): cv.enum(
            INPUT_FORMATS, lower=True
        ),
```

Add to `to_code` (after the `set_mic_channel` line):

```python
    cg.add(var.set_mic_mono16(config[CONF_INPUT_FORMAT]))
```

- [ ] **Step 2: Add the setter and member in `va_client.h`**

After the `set_mic_channel` setter (`va_client.h:71`):

```cpp
  void set_mic_mono16(bool m) { mic_mono16_ = m; }
```

Next to the `mic_channel_` member (`va_client.h:191`):

```cpp
  bool mic_mono16_{false};
```

- [ ] **Step 3: Branch the mic reader in `va_client.cpp`**

Replace the body of `on_mic_data_` from the stereo-int32 comment (`va_client.cpp:649`) through the end of the `mono_buf_` fill loop (`va_client.cpp:664`) with a branch. The `mono16` path forwards the samples unchanged; the `stereo32` path is the existing logic verbatim.

```cpp
  const int16_t *send_data;
  size_t send_samples;

  if (this->mic_mono16_) {
    // Plain codec (e.g. ES8311): frames are already int16 mono PCM. The i2s
    // tail can leave an odd trailing byte mid-frame; truncate to whole samples.
    send_samples = samples.size() / sizeof(int16_t);
    if (send_samples == 0)
      return;
    send_data = reinterpret_cast<const int16_t *>(samples.data());
  } else {
    // i2s_mics yields interleaved stereo int32 frames: [L0_low,L0_high, R0_low,R0_high, L1..].
    // Each frame = 8 bytes (2ch × 4 bytes). We want one channel converted to
    // int16 mono. Real audio sits in the high 16 bits (ADC pads up to int32).
    if (samples.size() < 8)
      return;
    const auto *in32 = reinterpret_cast<const int32_t *>(samples.data());
    size_t total_int32 = samples.size() / 4;
    size_t mono_samples = total_int32 / 2;
    size_t offset = this->mic_channel_ & 0x1;
    this->mono_buf_.resize(mono_samples);
    for (size_t i = 0; i < mono_samples; i++) {
      int32_t s = in32[i * 2 + offset];
      this->mono_buf_[i] = static_cast<int16_t>(s >> 16);
    }
    send_data = this->mono_buf_.data();
    send_samples = this->mono_buf_.size();
  }

  auto handle = static_cast<esp_websocket_client_handle_t>(this->ws_handle_);
  esp_websocket_client_send_bin(handle, reinterpret_cast<const char *>(send_data),
                                static_cast<int>(send_samples * sizeof(int16_t)),
                                10 / portTICK_PERIOD_MS);
```

(The existing `esp_websocket_client_send_bin` call at `va_client.cpp:670-672` is now part of the replacement above — do not leave the old copy.)

- [ ] **Step 4: Validate Voice PE config still parses (no behavior change)**

Run: `esphome config home-assistant-voice.va-direct.yaml`
Expected: parses with no errors; `va_client` defaults `input_format` to `stereo32` (the option is not set in that file, so behavior is unchanged).

- [ ] **Step 5: Commit (only if the user asks)**

```bash
git add esphome/components/va_client/__init__.py esphome/components/va_client/va_client.h esphome/components/va_client/va_client.cpp
git commit -m "feat(va_client): add mono16 input_format for plain codecs"
```

---

### Task 2: Create `atom-echo-s3r.va-direct.yaml`

**Files:**
- Create: `atom-echo-s3r.va-direct.yaml`

**Interfaces:**
- Consumes: `va_client` `input_format: mono16` (Task 1); existing `secrets.yaml` keys `va_url`, `va_device_token`, WiFi creds.
- Produces: a flashable board config.

- [ ] **Step 1: Write the config**

Create `atom-echo-s3r.va-direct.yaml` with the minimal thin-client wiring (full content):

```yaml
substitutions:
  name: atom-echo-s3r
  friendly_name: Atom Echo S3R
  # va-direct: voice-assistant WS endpoint (mDNS on LAN). Override in secrets if needed.
  va_url: "ws://va.local:3001/voice"

esphome:
  name: ${name}
  friendly_name: ${friendly_name}
  name_add_mac_suffix: true
  min_version: 2026.6.0
  on_boot:
    priority: 600
    then:
      - switch.turn_on: speaker_amp
      - micro_wake_word.start:

esp32:
  board: esp32-s3-devkitc-1
  cpu_frequency: 240MHz
  variant: esp32s3
  flash_size: 8MB
  framework:
    type: esp-idf

psram:
  mode: octal
  speed: 80MHz

logger:
api:
  encryption:
    key: !secret api_encryption_key
ota:
  - platform: esphome
    password: !secret ota_password
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  ap:
captive_portal:

i2c:
  - id: bus_a
    sda: GPIO45
    scl: GPIO4
    frequency: 400kHz

audio_dac:
  - platform: es8311
    id: es8311_dac
    address: 0x18
    sample_rate: 16000
    bits_per_sample: 16bit

i2s_audio:
  - id: i2s_shared
    i2s_lrclk_pin: GPIO11
    i2s_bclk_pin: GPIO17
    i2s_mclk_pin: GPIO18

microphone:
  - platform: i2s_audio
    id: echo_mic
    i2s_din_pin: GPIO0
    adc_type: external
    pdm: false
    sample_rate: 16000
    bits_per_sample: 16bit
    channel: mono
    i2s_mode: primary
    i2s_audio_id: i2s_shared

speaker:
  - platform: i2s_audio
    id: i2s_speaker
    dac_type: external
    i2s_dout_pin: GPIO48
    sample_rate: 16000
    bits_per_sample: 16bit
    channel: mono
    i2s_mode: primary
    i2s_audio_id: i2s_shared
  - platform: resampler
    id: va_speaker
    output_speaker: i2s_speaker
    sample_rate: 16000
    bits_per_sample: 16

switch:
  - platform: gpio
    id: speaker_amp
    pin: GPIO3
    restore_mode: ALWAYS_OFF
    internal: true

micro_wake_word:
  models:
    - model: okay_nabu
    - model: hey_jarvis
    - model: stop
  on_wake_word_detected:
    - if:
        condition:
          lambda: 'return wake_word == "stop";'
        then:
          - lambda: id(va).interrupt();
        else:
          - lambda: id(va).start_session();

external_components:
  - source:
      type: local
      path: esphome/components
    components: [va_client, udp_log]

va_client:
  id: va
  url: ${va_url}
  token: !secret va_device_token
  microphone: echo_mic
  mic_channel: 0
  input_format: mono16
  speaker: va_speaker
  on_repeated_failure:
    - logger.log: "va_client: repeated connection failure"

binary_sensor:
  - platform: gpio
    id: action_button
    pin:
      number: GPIO41
      inverted: true
      mode:
        input: true
        pullup: true
    on_press:
      - lambda: id(va).interrupt();
```

- [ ] **Step 2: Validate the config parses**

Run: `esphome config atom-echo-s3r.va-direct.yaml`
Expected: parses with no schema errors. Resolve any rejected option against the installed ESPHome version (see Task 3 — these are the likely break points).

- [ ] **Step 3: Commit (only if the user asks)**

```bash
git add atom-echo-s3r.va-direct.yaml
git commit -m "feat: add Atom Echo S3R thin-client config"
```

---

### Task 3: Verify build + reconcile ESPHome API drift

**Files:**
- Modify (if needed): `atom-echo-s3r.va-direct.yaml`

**Interfaces:**
- Consumes: Tasks 1–2.
- Produces: a config that compiles.

This task exists because the yaml was written blind. The following are the
known-uncertain spots; confirm each against the installed ESPHome version and
the M5 schematic, fixing the yaml in place.

- [ ] **Step 1: Compile**

Run: `esphome compile atom-echo-s3r.va-direct.yaml`
Expected: clean build producing a firmware binary.

- [ ] **Step 2: Reconcile likely break points if compile fails**

Check, in priority order:

1. **`va_client` C++ API names (confirmed against `va_client.h`).** The wake-word and button lambdas call `id(va)->start_session()` and `id(va)->send_interrupt()` (pointer arrow, and `send_interrupt` — NOT `interrupt`). These match the public methods in `esphome/components/va_client/va_client.h`. `start_session()` already barges in (bridge cancels in-flight reply on `start`; flush folded in as backstop), so no separate barge-in call is needed for the minimal config.
2. **ES8311 full-duplex / mic capture.** If the `es8311` `audio_dac` component does not enable the ADC path, the mic will be silent even though the build succeeds — this surfaces on-device, not at compile. Note it for the hardware test (spec risk #1).
3. **`microphone` / `speaker` i2s options.** `dac_type` / `adc_type`, `channel`, and `i2s_mode` keys vary across ESPHome versions; if a key is rejected, match the form used in `home-assistant-voice.va-direct.yaml` for that version.
4. **`resampler` speaker platform.** Confirm the `resampler` speaker platform name and that it accepts a differing input rate; if `va_client` feeding 24 kHz into a 16 kHz-declared resampler is rejected, the resampler input rate is driven by `va_client`'s stream info (24 kHz) and `sample_rate` here is the output — adjust per the component's schema.
5. **`api` / `ota` / `psram` secrets and keys.** Ensure `secrets.yaml` has `api_encryption_key`, `ota_password`, `wifi_ssid`, `wifi_password`, `va_device_token`.

- [ ] **Step 3: Confirm Voice PE regression-free**

Run: `esphome config home-assistant-voice.va-direct.yaml`
Expected: still parses; `va_client` there uses default `stereo32`.

- [ ] **Step 4: Commit any fixes (only if the user asks)**

```bash
git add atom-echo-s3r.va-direct.yaml
git commit -m "fix: reconcile Atom Echo S3R config with ESPHome API"
```

---

### Task 4: Document the new board

**Files:**
- Modify: `CLAUDE.md` (this repo) — add the Atom Echo S3R config to the Configs table and note the `input_format` option.
- Modify: `../CLAUDE.md` (umbrella) — only if the device joins the running stack (defer until flashed).

- [ ] **Step 1: Add a row to the Configs table in `CLAUDE.md`**

In the `## Configs` table, add:

```markdown
| `atom-echo-s3r.va-direct.yaml` | Thin-client config for the M5Stack Atom Echo S3R (ES8311 mono codec, no XMOS). Uses `va_client` with `input_format: mono16`. |
```

- [ ] **Step 2: Note the `input_format` option in the component section**

In `## Custom component: esphome/components/va_client/`, update the `__init__.py` row to mention `input_format` (`stereo32` default / `mono16`).

- [ ] **Step 3: Commit (only if the user asks)**

```bash
git add CLAUDE.md
git commit -m "docs: document Atom Echo S3R config and input_format option"
```

---

## Self-Review

**Spec coverage:**
- `va_client` generalization (`input_format`) → Task 1. ✓
- New `atom-echo-s3r.va-direct.yaml` minimal thin client → Task 2. ✓
- Audio format chain (16 kHz bus + resampler) → Task 2 speaker block. ✓
- Risk #1 (ES8311 full-duplex) → flagged in Task 3 Step 2.2, deferred to on-device. ✓
- Risk #2 (single-bus rate) → Task 2 (resampler) + Task 3 Step 2.3/2.4. ✓
- Risk #3 (no AEC) → no code action; accepted in spec. ✓
- Risk #4 (pinout/no LED) → no LED in config; pinout verify noted in Task 3 Step 2. ✓
- Voice PE untouched → Task 1 Step 4 + Task 3 Step 3 (default `stereo32`). ✓

**Placeholder scan:** No TBD/TODO; all code shown in full. Task 3 is explicitly a reconciliation task, with concrete check items, not a placeholder.

**Type consistency:** `set_mic_mono16(bool)` / `mic_mono16_` used consistently across `__init__.py`, `.h`, `.cpp`. Wake-word and button lambdas use `id(va)->start_session()` / `id(va)->send_interrupt()`, verified against `va_client.h`. micro_wake_word requires a `microphone:` sub-block (mirrored from the Voice PE config: `microphone: echo_mic`, `channels: 1`, `gain_factor: 4`).
