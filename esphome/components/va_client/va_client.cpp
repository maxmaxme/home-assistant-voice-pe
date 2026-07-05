/*
 * va_client — direct-streaming WebSocket client component implementation.
 *
 * New in this fork of Home Assistant Voice: Preview Edition
 * (esphome/home-assistant-voice-pe): replaces the stock ESPHome
 * `voice_assistant` component with a thin client that streams PCM16 audio
 * directly to a voice-assistant backend over a WebSocket. Added 2026.
 *
 * IMPERATIVE SHELL. The control plane (state machine, watchdog time math,
 * server-JSON handling) lives in va_core.h and is host-tested in tests/host/.
 * This file owns the platform: esp_websocket_client, the mic/speaker paths,
 * the PSRAM audio ring, and ESPHome trigger firing. Threading contract:
 * the WS task touches ONLY the binary-audio path (handle_binary_); all text
 * frames and connect/disconnect events are marshalled onto the ESPHome main
 * loop via defer() before the core sees them, so the core is single-threaded.
 *
 * Copyright (C) 2026 maxmaxme.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details. You should have received a copy of the GNU General Public
 * License along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "va_client.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/audio/audio.h"

#include <cstring>

#include <esp_websocket_client.h>
#include <esp_event.h>
#include <esp_heap_caps.h>

namespace esphome {
namespace va_client {

static const char *const TAG = "va_client";

#ifdef USE_VA_CLIENT_DIAGNOSTICS
// Loud-garbage detector thresholds. A PCM16 chunk where most samples sit near
// full scale is noise, not speech (speech peaks but doesn't sustain near
// ±32767). We measure at two points — the chunk just received over the WS, and
// the slice about to be handed to the speaker — so we can tell whether garbage
// arrived over the wire (upstream / OpenAI) or was introduced device-side
// (ring / scaling / a cross-task race) between receipt and playback.
static constexpr int32_t kNoiseLevel = 19660;  // ~0.6 * 32767
static constexpr float kNoiseRatio = 0.5f;     // share of samples above level
#endif

// Trigger constructors — registered against their parent VaClient so the
// yaml-generated trigger lifecycle stays standard. Definitions live here
// rather than inline in the header to break the otherwise-circular
// dependency between trigger ctor and VaClient::add_*_trigger().
OnPhaseTrigger::OnPhaseTrigger(VaClient *parent) {
  parent->add_on_phase_trigger(this);
}
OnRepeatedFailureTrigger::OnRepeatedFailureTrigger(VaClient *parent) {
  parent->add_on_repeated_failure_trigger(this);
}
OnFollowupOpenedTrigger::OnFollowupOpenedTrigger(VaClient *parent) {
  parent->add_on_followup_opened_trigger(this);
}

// Free-function trampoline. esp-idf event registration takes a C function
// pointer; we recover the VaClient* from the user_data slot.
static void va_ws_event_handler(void *handler_args, esp_event_base_t /*base*/, int32_t event_id, void *event_data) {
  auto *self = static_cast<VaClient *>(handler_args);
  if (self == nullptr)
    return;
  self->on_ws_event(event_id, event_data);
}

void VaClient::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VA Client...");

  if (this->mic_ != nullptr) {
    this->mic_->add_data_callback(
        [this](const std::vector<uint8_t> &data) { this->on_mic_data_(data); });
  } else {
    ESP_LOGE(TAG, "Microphone not configured");
  }

  // Allocate the audio ring buffer in PSRAM (8 MB available, internal RAM
  // is only 320 KB and we don't want to starve wifi/mww).
  this->audio_buf_ = static_cast<uint8_t *>(
      heap_caps_malloc(kAudioBufBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->audio_buf_ == nullptr) {
    // Without the ring every reply is silently dropped (handle_binary_
    // no-ops) while the rest of the component pretends to work. Fail loudly
    // instead of limping (review L3).
    ESP_LOGE(TAG, "Failed to allocate %u-byte audio buffer in PSRAM", (unsigned) kAudioBufBytes);
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "Allocated %u-byte audio ring buffer in PSRAM", (unsigned) kAudioBufBytes);

  // Tell the resampler what format we'll feed it. The resampler converts to
  // its yaml-configured output format (48k 16-bit) before passing to the
  // mixer → i2s leaf. Start the speaker task once so play() calls just push
  // into its ring buffer instead of racing to re-create the i2s channel.
  if (this->speaker_ != nullptr) {
    audio::AudioStreamInfo info(/*bits_per_sample=*/16, /*channels=*/1, /*sample_rate=*/24000);
    this->speaker_->set_audio_stream_info(info);
    this->speaker_->start();
  }

  this->connect_();
}

// ---- Core plumbing ----------------------------------------------------------

size_t VaClient::audio_fill_snapshot_() {
  portENTER_CRITICAL(&this->ring_mux_);
  size_t fill = this->audio_fill_;
  portEXIT_CRITICAL(&this->ring_mux_);
  return fill;
}

template<typename F> void VaClient::run_core_(F &&f) {
  // A fresh local list per event: no alloc while empty (the common tick),
  // and execute_actions_ can safely trigger nested core runs (a failed
  // ConnectWs schedules the next reconnect) without clobbering this one.
  Actions actions;
  f(this->core_, actions);
  this->execute_actions_(actions);
}

void VaClient::execute_actions_(const Actions &actions) {
  for (const auto &a : actions) {
    switch (a.type) {
      case Action::Type::SendStart: {
        static const char kMsg[] = "{\"type\":\"start\"}";
        this->send_text_frame_(kMsg, sizeof(kMsg) - 1);
        ESP_LOGD(TAG, "send start — WS msg sent");
        break;
      }
      case Action::Type::SendInterrupt: {
        static const char kMsg[] = "{\"type\":\"interrupt\"}";
        this->send_text_frame_(kMsg, sizeof(kMsg) - 1);
        ESP_LOGD(TAG, "send interrupt — WS msg sent");
        break;
      }
      case Action::Type::EmitPhase: {
#ifdef USE_VA_CLIENT_DIAGNOSTICS
        // Latency anchors: the core emits "listening"/"thinking" for the
        // server-confirmed transitions; the ==0 guard plus turn_t_wake_
        // gating (set AFTER start_session's own emission executes) keeps
        // the anchors on the server-driven edges, as before the split.
        if (this->turn_t_wake_ != 0) {
          if (this->turn_t_listening_ == 0 && std::strcmp(a.phase, "listening") == 0) {
            this->turn_t_listening_ = millis();
          } else if (this->turn_t_thinking_ == 0 && std::strcmp(a.phase, "thinking") == 0) {
            this->turn_t_thinking_ = millis();
          }
        }
#endif
        ESP_LOGI(TAG, "phase -> %s (state=%d)", a.phase, (int) this->core_.state());
        this->emit_phase_(a.phase);
        break;
      }
      case Action::Type::FireFollowupOpened: {
        ESP_LOGI(TAG, "chimed follow-up — firing on_followup_opened");
        this->defer([this]() {
          for (auto *t : this->followup_opened_triggers_) {
            t->trigger();
          }
        });
        break;
      }
      case Action::Type::FireRepeatedFailure: {
        ESP_LOGW(TAG, "firing on_repeated_failure");
        this->defer([this]() {
          for (auto *t : this->repeated_failure_triggers_) {
            t->trigger();
          }
        });
        break;
      }
      case Action::Type::FlushAudioQueue: {
        this->flush_audio_queue();
        break;
      }
      case Action::Type::ConnectWs: {
        this->connect_();
        break;
      }
      case Action::Type::LogTurnStats: {
#ifdef USE_VA_CLIENT_DIAGNOSTICS
        if (this->turn_t_wake_ != 0) {
          uint32_t now = millis();
          auto fmt = [](uint32_t from, uint32_t to) -> std::string {
            if (from == 0 || to == 0 || to < from)
              return "?";
            return std::to_string(to - from) + "ms";
          };
          ESP_LOGI(TAG,
                   "turn latency: wake→listening=%s listening→thinking=%s "
                   "thinking→first_audio=%s first_audio→played_out=%s "
                   "total=%s",
                   fmt(this->turn_t_wake_, this->turn_t_listening_).c_str(),
                   fmt(this->turn_t_listening_, this->turn_t_thinking_).c_str(),
                   fmt(this->turn_t_thinking_, this->turn_t_first_audio_out_).c_str(),
                   fmt(this->turn_t_first_audio_out_, now).c_str(),
                   fmt(this->turn_t_wake_, now).c_str());
          if (this->ws_gap_count_ > 0 || this->clipped_samples_ > 0 ||
              this->underrun_logged_this_turn_) {
            ESP_LOGW(TAG,
                     "turn audio: ws_gaps=%u (max=%ums) clipped_samples=%u underrun=%s",
                     (unsigned) this->ws_gap_count_,
                     (unsigned) this->ws_gap_max_ms_,
                     (unsigned) this->clipped_samples_,
                     this->underrun_logged_this_turn_ ? "yes" : "no");
          }
          this->turn_t_wake_ = 0;  // mark turn as logged
        }
#endif
        break;
      }
      case Action::Type::CloseWsProtocolError: {
        // Server hello announced an audio format we can't play — its binary
        // frames would be decoded as PCM16 garbage (loud noise). Close and
        // let the reconnect machinery retry; harmless-loop until the server
        // is fixed. Safe here: execute_actions_ runs on the main loop, never
        // on the WS event task (where close is forbidden).
        ESP_LOGE(TAG, "server audioOut format unsupported (only \"pcm\") — closing WS");
        if (this->ws_handle_ != nullptr) {
          esp_websocket_client_close(
              static_cast<esp_websocket_client_handle_t>(this->ws_handle_), pdMS_TO_TICKS(1000));
        }
        break;
      }
      case Action::Type::WarnProtoMismatch: {
        ESP_LOGW(TAG,
                 "server /voice protocol version differs from this firmware's (%u) — "
                 "update firmware and backend in lockstep; continuing best-effort",
                 (unsigned) VaCore::kProtoVersion);
        break;
      }
    }
  }
}

void VaClient::send_text_frame_(const char *json, size_t len) {
  if (!this->ws_connected_ || this->ws_handle_ == nullptr) {
    ESP_LOGW(TAG, "send_text_frame_: WS not connected");
    return;
  }
  auto handle = static_cast<esp_websocket_client_handle_t>(this->ws_handle_);
  // Bounded timeout (review M4): this runs on the main loop, and a half-dead
  // link with portMAX_DELAY would wedge the whole firmware (LED, wake word,
  // watchdog). On failure just log — the ping-pong timeout tears the link
  // down and the reconnect machinery recovers.
  int sent = esp_websocket_client_send_text(handle, json, static_cast<int>(len),
                                            pdMS_TO_TICKS(500));
  if (sent < 0) {
    ESP_LOGE(TAG, "WS text send failed/timed out — link unhealthy");
  }
}

void VaClient::emit_phase_(const std::string &phase) {
  // Fired via defer() so yaml automations run from a clean loop context
  // rather than re-entrantly inside whatever lambda called into the core
  // (same marshalling the pre-split code used).
  std::string phase_copy = phase;
  this->defer([this, phase_copy]() {
    for (auto *t : this->phase_triggers_) {
      t->trigger(phase_copy);
    }
  });
}

// ---- Main loop ---------------------------------------------------------------

void VaClient::loop() {
  this->drain_audio_ring_();

  // Control-plane tick: the core fires whatever deadline came due (watchdogs,
  // reconnect backoff, follow-up windows) and drives the WaitingDrain →
  // finish-drain decision from the fill / downstream-chain snapshot.
  //
  // The drained signal walks the chain (resampler ring + mixer source ring)
  // via has_buffered_data() — we use this instead of is_stopped() because our
  // mixer sources are configured `timeout: never` and stay RUNNING forever.
  // It does *not* cover the i2s 500ms ring + ~100ms DAC tail downstream of
  // the mixer, so the drain completes ~500ms before true silence; for the
  // LED that's imperceptible, and the chimed follow-up's yaml wait absorbs it.
  const size_t fill = this->audio_fill_snapshot_();
  const bool speaker_drained =
      (this->speaker_ != nullptr) && !this->speaker_->has_buffered_data();
  this->run_core_([now = millis(), fill, speaker_drained](VaCore &core, Actions &out) {
    core.on_tick(now, fill, speaker_drained, out);
  });
}

void VaClient::drain_audio_ring_() {
  // Drain the audio ring buffer into the speaker. speaker.play() accepts
  // only what fits in its own ring (returns the count actually queued).
  if (this->speaker_ != nullptr && this->audio_buf_ != nullptr) {
    // Snapshot ring state under the lock — head/tail/fill are all
    // mutated from the WS task on the other core.
    portENTER_CRITICAL(&this->ring_mux_);
    size_t head = this->audio_head_;
    size_t tail = this->audio_tail_;
    size_t fill = this->audio_fill_;
    portEXIT_CRITICAL(&this->ring_mux_);
    if (fill > 0) {
      const uint32_t now_ms = millis();
      // Resampler cold-start SILENCE-PRIME (crackle fix). When the chain is
      // cold (post speaker.stop, or nothing fed for > kChainColdMs, or never
      // fed) feed kChainPrimeMs of silence BEFORE the first real sample so the
      // resampler's windowed-sinc FIR settles to a clean zero output and the
      // reply doesn't open with a startup click. Real audio waits safely in
      // PSRAM (and builds a small cushion) until priming completes. See the
      // header note for why neither signal can mis-fire mid-speech.
      const bool resampler_cold = this->speaker_->is_stopped() ||
                                  this->last_fed_ms_ == 0 ||
                                  (now_ms - this->last_fed_ms_) > kChainColdMs;
      if (this->chain_prime_remaining_ == 0 && resampler_cold) {
        this->chain_prime_remaining_ = kChainPrimeBytes;
        ESP_LOGD(TAG, "resampler cold — priming %u bytes of silence before reply",
                 (unsigned) this->chain_prime_remaining_);
      }
      if (this->chain_prime_remaining_ > 0) {
        static const uint8_t kSilence[480] = {0};  // 10ms @24k mono16; fed in chunks
        size_t want = std::min(this->chain_prime_remaining_, sizeof(kSilence));
        size_t fed = this->speaker_->play(kSilence, want);
        if (fed > 0) {
          this->chain_prime_remaining_ -= fed;
          this->last_fed_ms_ = now_ms;  // count silence as "fed" so the cold-check clears
        }
        // Hold real-audio drain until the chain is warmed; re-enter loop()
        // next tick to continue/finish priming.
        return;
      }
      // Jitter buffer priming gate. After the ring was empty (reply start or a
      // post-underflow gap) hold playback until either the prebuffer cushion
      // has accumulated (fill >= target) or a deadline elapses (so real-time,
      // non-burst audio still starts promptly). The cushion lets the downstream
      // chain ride out a network gap without drying out → no crackle.
      if (this->playback_priming_) {
        if (fill >= kPlaybackPrebufferBytes ||
            (now_ms - this->prime_started_ms_) >= kPlaybackPrebufferMs) {
          this->playback_priming_ = false;
          ESP_LOGD(TAG, "prebuffer ready (%u bytes) — playback start", (unsigned) fill);
        } else {
          return;  // keep accumulating; don't drain (and don't false-flag underrun)
        }
      }
#ifdef USE_VA_CLIENT_DIAGNOSTICS
      // Detector 3: downstream underrun. If the resampler/mixer/i2s chain
      // ran out of bytes to play while we *still* have PSRAM queued,
      // something hiccupped downstream — the user hears silence or a
      // brief stuck-sample glitch. Log the first occurrence per reply so
      // we know whether bad audio in a turn correlates with this.
      if (!this->underrun_logged_this_turn_ && this->playback_started_this_turn_ &&
          !this->speaker_->has_buffered_data()) {
        ESP_LOGW(TAG, "downstream underrun: %u bytes queued in PSRAM but speaker chain is dry",
                 (unsigned) fill);
        this->underrun_logged_this_turn_ = true;
      }
#endif
      // Contiguous slice we can hand to play() without copying: from head
      // to either the end of the buffer or the tail.
      size_t contiguous = (head < tail) ? (tail - head) : (kAudioBufBytes - head);
      if (contiguous > fill)
        contiguous = fill;
      // play() runs OUTSIDE the critical section: it can take milliseconds
      // (resampler ring may be full, mixer blocks). Holding ring_mux_
      // across it would block the writer and cause audio underrun.
      size_t accepted = this->speaker_->play(this->audio_buf_ + head, contiguous);
      if (accepted > 0) {
        this->last_fed_ms_ = millis();  // keep the chain "warm" for cold-detection
        portENTER_CRITICAL(&this->ring_mux_);
        this->audio_head_ = (this->audio_head_ + accepted) % kAudioBufBytes;
        this->audio_fill_ -= accepted;
        portEXIT_CRITICAL(&this->ring_mux_);
#ifdef USE_VA_CLIENT_DIAGNOSTICS
        this->playback_started_this_turn_ = true;
#endif
        static uint32_t dbg_last = 0;
        uint32_t now = millis();
        if (now - dbg_last >= 500) {
          ESP_LOGD(TAG, "drained %u bytes (%u still queued)", (unsigned) accepted,
                   (unsigned) (fill - accepted));
          dbg_last = now;
        }
#ifdef USE_VA_CLIENT_DIAGNOSTICS
        // Scan the slice we just handed to the speaker. [head, head+accepted)
        // is contiguous (contiguous never wraps), so a linear read is safe.
        // If this is noise-like but the matching incoming chunk was NOT, the
        // garbage was introduced device-side between receipt and playback
        // (ring write / scaling / a cross-task race) rather than over the wire.
        {
          const int16_t *pb = reinterpret_cast<const int16_t *>(this->audio_buf_ + head);
          size_t ns = accepted / 2;
          uint32_t loud = 0, scanned = 0;
          for (size_t i = 0; i < ns; i += 4) {  // sample every 4th frame
            if (pb[i] > kNoiseLevel || pb[i] < -kNoiseLevel) loud++;
            scanned++;
          }
          if (scanned > 0 && static_cast<float>(loud) / static_cast<float>(scanned) >= kNoiseRatio) {
            static uint32_t noise_pb_last = 0;
            if (now - noise_pb_last >= 200) {
              ESP_LOGW(TAG,
                       "playback noise-like slice: %u/%u near full scale (state=%d fill=%u)",
                       (unsigned) loud, (unsigned) scanned, (int) this->core_.state(),
                       (unsigned) fill);
              noise_pb_last = now;
            }
          }
        }
#endif
      }
    }
  }
}

// ---- WebSocket ---------------------------------------------------------------

void VaClient::connect_() {
  if (this->ws_handle_ != nullptr) {
    // Already initialised; (re)start. Stop first (review M1): after an
    // abnormal teardown the client can linger in a state where start()
    // returns ESP_ERR_INVALID_STATE; stop on an already-stopped client is a
    // harmless error return. Safe here — connect_ only ever runs on the main
    // loop, never on the WS event task (where stop is forbidden).
    auto handle = static_cast<esp_websocket_client_handle_t>(this->ws_handle_);
    esp_websocket_client_stop(handle);
    esp_err_t err = esp_websocket_client_start(handle);
    if (err != ESP_OK) {
      // Without this the retry chain would end here forever (no WS event
      // fires for a failed local start). Mirror the first-init path:
      // schedule the next backoff attempt.
      ESP_LOGE(TAG, "esp_websocket_client_start (reconnect) failed: %d", (int) err);
      this->run_core_([now = millis()](VaCore &core, Actions &out) {
        core.on_connect_failed(now, out);
      });
    }
    return;
  }

  // Keep the header string alive for the entire lifetime of the client; the
  // config struct only stores a pointer.
  this->auth_header_ = "Authorization: Bearer " + this->token_ + "\r\n";

  esp_websocket_client_config_t cfg = {};
  cfg.uri = this->url_.c_str();
  cfg.headers = this->auth_header_.c_str();
  cfg.disable_auto_reconnect = true;  // we drive reconnects ourselves with exponential backoff
  cfg.reconnect_timeout_ms = 5000;    // ignored because disable_auto_reconnect=true
  // Half-dead-link detection (review M5): without a pong deadline a link
  // that silently stops passing traffic (AP reboot, NAT drop) never emits a
  // disconnect event and the device hangs "connected" forever. With it the
  // client tears the transport down and our reconnect machinery takes over.
  cfg.pingpong_timeout_sec = 30;

  esp_websocket_client_handle_t handle = esp_websocket_client_init(&cfg);
  if (handle == nullptr) {
    ESP_LOGE(TAG, "esp_websocket_client_init failed");
    this->run_core_([now = millis()](VaCore &core, Actions &out) {
      core.on_connect_failed(now, out);
    });
    return;
  }
  this->ws_handle_ = handle;

  esp_err_t err = esp_websocket_register_events(handle, WEBSOCKET_EVENT_ANY, va_ws_event_handler, this);
  if (err != ESP_OK) {
    // Starting anyway would mean a client we never hear from — no connected
    // flag, no text frames, no reconnect events. Fail loudly instead of
    // limping (review L3).
    ESP_LOGE(TAG, "esp_websocket_register_events failed: %d", (int) err);
    esp_websocket_client_destroy(handle);
    this->ws_handle_ = nullptr;
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "Connecting to %s", this->url_.c_str());
  err = esp_websocket_client_start(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_websocket_client_start failed: %d", (int) err);
    this->run_core_([now = millis()](VaCore &core, Actions &out) {
      core.on_connect_failed(now, out);
    });
  }
}

void VaClient::on_ws_event(int32_t event_id, void *event_data) {
  // Runs on the esp-idf websocket task. ONLY the binary-audio path is
  // handled here; every control-plane event is marshalled onto the ESPHome
  // main loop via defer() so the core stays single-threaded. (defer() is
  // the one scheduler entry point that is safe cross-task, and was already
  // used from this task pre-split.)
  auto *data = static_cast<esp_websocket_event_data_t *>(event_data);
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED: {
      ESP_LOGI(TAG, "WS connected");
      // The shell-level connected flag flips immediately (mic gating and the
      // send guard read it), the core learns about it on the next loop.
      this->ws_connected_ = true;
      this->defer([this]() {
        this->run_core_([this](VaCore &core, Actions &out) {
          core.on_ws_connected(millis(), this->audio_fill_snapshot_(), out);
        });
      });
      break;
    }
    case WEBSOCKET_EVENT_DATA: {
      if (data == nullptr || data->data_ptr == nullptr || data->data_len <= 0)
        break;
      // op_code: 0x01 = text, 0x02 = binary, 0x00 = continuation of the prior
      // frame. esp_websocket_client splits long messages, so we must track the
      // type from the first chunk and feed continuations to the same handler.
      uint8_t op = data->op_code;
      bool is_binary;
      if (op == 0x01) {
        is_binary = false;
      } else if (op == 0x02) {
        is_binary = true;
      } else if (op == 0x00) {
        is_binary = this->last_data_was_binary_;
      } else {
        break;
      }
      this->last_data_was_binary_ = is_binary;
      if (is_binary) {
        // Audio plane: stays on the WS task (PSRAM ring write under the mux).
        this->handle_binary_(reinterpret_cast<const uint8_t *>(data->data_ptr),
                             static_cast<size_t>(data->data_len));
      } else {
        // Control plane: copy the frame and hand it to the core on the main
        // loop. Frames are tiny JSON objects (≤ ~64 bytes), so the copy is
        // negligible. The fill snapshot is taken at processing time, same as
        // the pre-split code reading audio_fill_ inside the handler.
        ESP_LOGD(TAG, "WS text: %.*s", (int) data->data_len, data->data_ptr);
        std::string frame(data->data_ptr, static_cast<size_t>(data->data_len));
        this->defer([this, frame]() {
          this->run_core_([this, &frame](VaCore &core, Actions &out) {
            core.on_server_text(frame.data(), frame.size(), millis(),
                                this->audio_fill_snapshot_(), out);
          });
        });
      }
      break;
    }
    case WEBSOCKET_EVENT_DISCONNECTED:
    case WEBSOCKET_EVENT_CLOSED:
    case WEBSOCKET_EVENT_ERROR: {
      if (this->ws_connected_) {
        ESP_LOGW(TAG, "WS disconnected (event %d)", (int) event_id);
      }
      this->ws_connected_ = false;
      // The core coalesces the multiple events esp_websocket_client emits per
      // failure (DISCONNECTED + CLOSED, sometimes ERROR) into one reconnect.
      this->defer([this]() {
        this->run_core_([this](VaCore &core, Actions &out) {
          core.on_ws_disconnected(millis(), this->audio_fill_snapshot_(), out);
        });
      });
      break;
    }
    default:
      break;
  }
}

// ---- Audio plane ---------------------------------------------------------------

void VaClient::handle_binary_(const uint8_t *data, size_t len) {
  if (this->speaker_ == nullptr || len < 2 || this->audio_buf_ == nullptr)
    return;
  // Drop binary frames unless the state expects reply audio (review L2):
  // stray audio in Idle/Listening/FollowupArmed would otherwise queue up and
  // blurt out at the start of the next reply. Same benign cross-task
  // single-byte enum read as is_mic_streaming() on the mic task.
  if (!this->core_.accepts_reply_audio()) {
    static uint32_t drop_log_last = 0;
    uint32_t now = millis();
    if (now - drop_log_last >= 1000) {
      ESP_LOGW(TAG, "dropping %u bytes of unexpected reply audio (state=%d)",
               (unsigned) len, (int) this->core_.state());
      drop_log_last = now;
    }
    return;
  }
#ifdef USE_VA_CLIENT_DIAGNOSTICS
  const uint32_t now_ms = millis();
  if (this->turn_t_first_audio_out_ == 0 && this->turn_t_wake_ != 0) {
    this->turn_t_first_audio_out_ = now_ms;
  }
  // Detector 1: WS frame inter-arrival jitter. Normal cadence is ~20 ms
  // per frame (OpenAI streams realtime). A gap > kWsGapWarnMs means the
  // bridge stalled, network blip, or OpenAI burst late — anything that
  // could starve the downstream chain. Log immediately so the gap is
  // adjacent to whatever the user reports hearing.
  if (this->last_binary_ms_ != 0) {
    const uint32_t gap = now_ms - this->last_binary_ms_;
    if (gap > kWsGapWarnMs) {
      this->ws_gap_count_++;
      if (gap > this->ws_gap_max_ms_) this->ws_gap_max_ms_ = gap;
      ESP_LOGW(TAG, "ws audio gap: %u ms (ring fill %u bytes)",
               (unsigned) gap, (unsigned) this->audio_fill_);
    }
  }
  this->last_binary_ms_ = now_ms;
#endif
  // PCM16 mono @ 24 kHz, append to ring buffer. loop() drains.
  // Snapshot audio_fill_ under the lock — it's modified by loop() on the
  // other core and we can't trust a torn read.
  size_t free_space;
  portENTER_CRITICAL(&this->ring_mux_);
  free_space = kAudioBufBytes - this->audio_fill_;
  portEXIT_CRITICAL(&this->ring_mux_);
  if (len > free_space) {
    ESP_LOGW(TAG, "audio buffer overflow: dropping %u bytes (have %u free of %u total)",
             (unsigned) (len - free_space), (unsigned) free_space, (unsigned) kAudioBufBytes);
    len = free_space;
    if (len == 0)
      return;
  }
  // Apply user-controlled volume from external_media_player before writing to
  // the ring. volume_ is set from yaml on every media_player volume / mute
  // change. With vol ≤ 1 there is no mathematical way to overflow int16, but
  // we keep a defensive saturation + clipped_samples_ counter so any future
  // gain re-introduction shows up in the per-turn summary instead of silently
  // distorting.
  size_t pairs = len / 2;
  if (pairs > 0) {
    auto *in = reinterpret_cast<const int16_t *>(data);
    // Playback path's OWN scratch — never mono_buf_, which the mic task owns.
    // Sharing it raced resize() across two tasks (see va_client.h note).
    this->play_buf_.resize(pairs);
    float vol = this->volume_;
    if (vol < 0.0f) vol = 0.0f;
    else if (vol > 1.0f) vol = 1.0f;
    // Q15 fixed point so the inner loop stays integer-only.
    int32_t scale = static_cast<int32_t>(vol * 32768.0f);
#ifdef USE_VA_CLIENT_DIAGNOSTICS
    uint32_t clipped = 0;
    uint32_t loud_in = 0;
#endif
    for (size_t i = 0; i < pairs; i++) {
#ifdef USE_VA_CLIENT_DIAGNOSTICS
      if (in[i] > kNoiseLevel || in[i] < -kNoiseLevel) loud_in++;
#endif
      int32_t v = (static_cast<int32_t>(in[i]) * scale) >> 15;
      if (v > 32767) {
        v = 32767;
#ifdef USE_VA_CLIENT_DIAGNOSTICS
        clipped++;
#endif
      } else if (v < -32768) {
        v = -32768;
#ifdef USE_VA_CLIENT_DIAGNOSTICS
        clipped++;
#endif
      }
      this->play_buf_[i] = static_cast<int16_t>(v);
    }
#ifdef USE_VA_CLIENT_DIAGNOSTICS
    this->clipped_samples_ += clipped;
    // If the chunk we just RECEIVED over the WS is already noise-like, the
    // garbage came over the wire (upstream / bridge / OpenAI), not from our
    // playback path. mic_streaming flags the cross-task-overlap case.
    if (pairs > 0 && static_cast<float>(loud_in) / static_cast<float>(pairs) >= kNoiseRatio) {
      static uint32_t noise_in_last = 0;
      uint32_t now = millis();
      if (now - noise_in_last >= 200) {
        ESP_LOGW(TAG,
                 "incoming noise-like chunk: %u/%u samples near full scale "
                 "(state=%d fill=%u mic_streaming=%d)",
                 (unsigned) loud_in, (unsigned) pairs, (int) this->core_.state(),
                 (unsigned) this->audio_fill_, (int) this->core_.is_mic_streaming());
        noise_in_last = now;
      }
    }
#endif
    data = reinterpret_cast<const uint8_t *>(this->play_buf_.data());
    // len is unchanged (pairs * 2 == len rounded down; trailing odd byte ignored).
    len = pairs * 2;
  }
  // Two-part write: from tail to end, then wrap to start.
  // We need a stable snapshot of audio_tail_ for the memcpy destination,
  // then commit tail + fill atomically with the writes so the reader on
  // the other core never sees a new tail before the memcpy completed.
  // Doing the memcpy *inside* the critical section is the simplest way
  // to guarantee that ordering — len is at most a few KB per WS frame
  // and PSRAM memcpy is ~10–20 µs, well under any audio deadline.
  portENTER_CRITICAL(&this->ring_mux_);
  const bool was_empty = (this->audio_fill_ == 0);
  size_t tail = this->audio_tail_;
  size_t first = std::min(len, kAudioBufBytes - tail);
  std::memcpy(this->audio_buf_ + tail, data, first);
  if (first < len) {
    std::memcpy(this->audio_buf_, data + first, len - first);
  }
  this->audio_tail_ = (tail + len) % kAudioBufBytes;
  this->audio_fill_ += len;
  portEXIT_CRITICAL(&this->ring_mux_);
  // Jitter buffer: arm priming only when the ring was empty AND the downstream
  // chain is dry — a true reply start or a real underflow. Mid-reply the ring
  // routinely flips empty (loop() drains each WS clump on arrival) while the
  // downstream chain still holds ~600 ms of audio; re-arming there would just
  // spam "prebuffer ready" and could hold a trailing chunk for the full
  // deadline. has_buffered_data() is a counter read, safe enough from the WS task.
  if (was_empty && !this->playback_priming_ && !this->speaker_->has_buffered_data()) {
    this->prime_started_ms_ = millis();
    this->playback_priming_ = true;
  }
  // No per-chunk log — fires 50+ times per reply at DEBUG and drowns the
  // log. The throttled drain log in loop() gives enough visibility into
  // queue depth.
}

void VaClient::on_mic_data_(const std::vector<uint8_t> &samples) {
  if (!this->ws_connected_ || this->ws_handle_ == nullptr)
    return;
  // Gate streaming on the wake word: if no session is active, drop frames.
  // Otherwise OpenAI Realtime's server VAD would trigger responses to any
  // random speech in the room — wake word would become decoration. The
  // session opens via start_session() (wake-word handler) and closes on
  // "phase":"idle" coming back from the server (response.done). The core's
  // mic predicate is a single-byte enum read — the same benign cross-task
  // read the pre-split current_state_ check was.
  if (!this->core_.is_mic_streaming())
    return;
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
    size_t mono_samples = total_int32 / 2;  // half belong to this channel
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
  // 10ms timeout (~portTICK_PERIOD_MS): if WS task is briefly busy we wait
  // a tick rather than dropping the frame and spamming "Could not lock"
  // errors. If we're swamped, we accept dropping rather than blocking mic.
  esp_websocket_client_send_bin(handle, reinterpret_cast<const char *>(send_data),
                                static_cast<int>(send_samples * sizeof(int16_t)),
                                10 / portTICK_PERIOD_MS);
}

// ---- YAML-callable actions (main loop) ------------------------------------------

void VaClient::start_session() {
  // Wake-word handler in yaml routes here. Opens the mic (core → Listening)
  // so on_mic_data_ starts forwarding to the server; the `start` the core
  // emits also carries barge-in (the bridge cancels any reply still in
  // flight and clears its input buffer), and FlushAudioQueue drops the old
  // reply's queued tail so only the new turn's audio plays.
#ifdef USE_VA_CLIENT_DIAGNOSTICS
  // Reset the per-turn anchors/detectors BEFORE running the core so the
  // wake's own "listening" emission doesn't set the server-confirm anchor;
  // turn_t_wake_ is stamped after, gating the anchors to the server edges.
  this->turn_t_wake_ = 0;
  this->turn_t_listening_ = 0;
  this->turn_t_thinking_ = 0;
  this->turn_t_first_audio_out_ = 0;
  this->last_binary_ms_ = 0;
  this->ws_gap_count_ = 0;
  this->ws_gap_max_ms_ = 0;
  this->clipped_samples_ = 0;
  this->underrun_logged_this_turn_ = false;
  this->playback_started_this_turn_ = false;
#endif
  this->run_core_([now = millis()](VaCore &core, Actions &out) {
    core.start_session(now, out);
  });
#ifdef USE_VA_CLIENT_DIAGNOSTICS
  this->turn_t_wake_ = millis();
#endif
}

void VaClient::send_interrupt() {
  if (!this->ws_connected_) {
    ESP_LOGW(TAG, "send_interrupt: WS not connected");
    // The core no-ops on its own connected_ guard; fall through so the two
    // stay in sync even if the flags briefly diverge across the defer gap.
  }
  this->run_core_([now = millis()](VaCore &core, Actions &out) {
    core.send_interrupt(now, out);
  });
}

void VaClient::prepare_barge_in() {
  // Barge-in: wake word fired during a reply. The yaml already flushed the
  // PSRAM ring + stopped the resampler; the core neutralises the state
  // machine (pins Idle, kills follow-up deadlines, drops pending modifiers)
  // so the imminent chime wait can't fire finish-drain → a stray follow-up
  // window. start_session() runs after the chime + echo-guard delay.
  this->core_.prepare_barge_in();
}

void VaClient::commit_followup_mic() {
  // Called from yaml's on_followup_opened automation once the chime has
  // finished playing AND the i2s tail has cleared (wait_until + delay).
  // The core no-ops unless it is still in FollowupArmed (a fresh wake, a
  // Stop or a new turn takes priority over a late commit).
  this->run_core_([now = millis()](VaCore &core, Actions &out) {
    core.commit_followup_mic(now, out);
  });
}

void VaClient::flush_audio_queue() {
  // The ring reset has to happen under the mux: handle_binary_ runs on the
  // WS task and could be mid-write — seeing head=tail=fill=0 partway through
  // would let it commit a tail/fill that index into a buffer we just cleared.
  size_t dropped;
  portENTER_CRITICAL(&this->ring_mux_);
  dropped = this->audio_fill_;
  this->audio_head_ = 0;
  this->audio_tail_ = 0;
  this->audio_fill_ = 0;
  portEXIT_CRITICAL(&this->ring_mux_);
  if (dropped > 0) {
    ESP_LOGD(TAG, "flushed %u bytes of queued playback audio", (unsigned) dropped);
  }
}

}  // namespace va_client
}  // namespace esphome
