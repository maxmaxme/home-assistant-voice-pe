# Home Assistant Voice: Preview Edition

> **This is a customized fork.** It builds the Voice PE as a thin
> client that streams audio directly to a `voice-assistant` backend
> over WebSocket — there is no Home Assistant `voice_assistant`
> pipeline on the audio path. The active config is
> [`home-assistant-voice.va-direct.yaml`](home-assistant-voice.va-direct.yaml).
> See
> [`docs/superpowers/specs/2026-05-25-voice-pe-direct-va-streaming-design.md`](docs/superpowers/specs/2026-05-25-voice-pe-direct-va-streaming-design.md)
> for the design and [`CLAUDE.md`](CLAUDE.md) for current
> implementation notes.

## Modifications in this fork

Forked from [`esphome/home-assistant-voice-pe`](https://github.com/esphome/home-assistant-voice-pe).
Changes relative to upstream (2026):

- **Added** `esphome/components/va_client/` — a custom ESPHome component
  (GPLv3) that replaces the stock `voice_assistant` audio path with a thin
  WebSocket client streaming PCM16 to a `voice-assistant` backend.
- **Added** `home-assistant-voice.va-direct.yaml` — the active config wiring
  `va_client` (mic, speaker, LED phases, error chime). The original
  `home-assistant-voice.yaml` is left untouched for upstream sync.

Licensing is unchanged from upstream — the ESPHome dual license (see
[`LICENSE`](LICENSE)): C++/runtime files are GPLv3, everything else MIT. The
new C++ component is therefore GPLv3.

This is the ESPHome source code of the [Home Assistant Voice: Preview Edition](https://www.home-assistant.io/voice-pe/).

See [the documentation](https://voice-pe.home-assistant.io/) for set up and troubleshooting.

If you need to re-install the firmware, [use this installer](https://esphome.github.io/home-assistant-voice-pe/).
