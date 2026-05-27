#include "va_client.h"

#include "esphome/core/log.h"
#include "esphome/components/audio/audio.h"

#include <ArduinoJson.h>

#include <cstring>

#include <esp_websocket_client.h>
#include <esp_event.h>
#include <esp_heap_caps.h>

namespace esphome {
namespace va_client {

static const char *const TAG = "va_client";

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
    ESP_LOGE(TAG, "Failed to allocate %u-byte audio buffer in PSRAM", (unsigned) kAudioBufBytes);
  } else {
    ESP_LOGCONFIG(TAG, "Allocated %u-byte audio ring buffer in PSRAM", (unsigned) kAudioBufBytes);
  }

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

void VaClient::loop() {
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
#ifdef USE_VA_CLIENT_DIAGNOSTICS
      // Detector 3: downstream underrun. If the resampler/mixer/i2s chain
      // ran out of bytes to play while we *still* have PSRAM queued,
      // something hiccupped downstream — the user hears silence or a
      // brief stuck-sample glitch. Log the first occurrence per reply so
      // we know whether bad audio in a turn correlates with this.
      if (!this->underrun_logged_this_turn_ && !this->speaker_->has_buffered_data()) {
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
        portENTER_CRITICAL(&this->ring_mux_);
        this->audio_head_ = (this->audio_head_ + accepted) % kAudioBufBytes;
        this->audio_fill_ -= accepted;
        portEXIT_CRITICAL(&this->ring_mux_);
        static uint32_t dbg_last = 0;
        uint32_t now = millis();
        if (now - dbg_last >= 500) {
          ESP_LOGD(TAG, "drained %u bytes (%u still queued)", (unsigned) accepted,
                   (unsigned) (fill - accepted));
          dbg_last = now;
        }
      }
    }
  }
  // While in WaitingDrain, monitor the downstream speaker chain
  // (resampler + mixer + i2s + DAC tail). Just because our PSRAM queue
  // is empty doesn't mean the user has heard the audio yet — and an
  // "сейчас посмотрю" preamble before a tool call would drain the ring
  // mid-turn, so we can't act on audio_fill_==0 alone.
  //
  // Primary signal: speaker_->has_buffered_data() — walks the chain
  // (resampler ring + mixer source ring) and returns false as soon as
  // both have drained. We use this instead of is_stopped() because the
  // resampler only transitions to STATE_STOPPED once its downstream
  // (mixer source) reports stopped, but our mixer sources are configured
  // `timeout: never` and stay RUNNING forever, so is_stopped() would
  // never fire.
  //
  // Note: this does *not* cover the i2s 500ms ring + ~100ms DAC tail
  // downstream of the mixer. We fire ~500ms before true silence. For
  // the LED that's imperceptible; for the request_follow_up chime,
  // yaml's wait_until !is_announcing + i2s tail delay already absorbs
  // any small overlap with the fading TTS tail.
  //
  // Fallback: kSpeakerStopTimeoutMs (3 s). If something wedges and the
  // speaker never drains, we still progress so the LED doesn't lock in
  // `replying` forever.
  if (this->current_state_ == State::WaitingDrain && this->audio_fill_ == 0) {
    const bool speaker_drained =
        (this->speaker_ != nullptr) && !this->speaker_->has_buffered_data();
    const bool timed_out =
        (millis() - this->state_entered_ms_) >= kSpeakerStopTimeoutMs;
    if (speaker_drained || timed_out) {
      if (timed_out && !speaker_drained) {
        ESP_LOGW(TAG,
                 "speaker still had buffered data after %u ms — "
                 "proceeding anyway (fallback)",
                 (unsigned) kSpeakerStopTimeoutMs);
      }
      this->finish_drain_();
    }
  }
}

void VaClient::connect_() {
  if (this->ws_handle_ != nullptr) {
    // Already initialised; just (re)start.
    esp_websocket_client_start(static_cast<esp_websocket_client_handle_t>(this->ws_handle_));
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

  esp_websocket_client_handle_t handle = esp_websocket_client_init(&cfg);
  if (handle == nullptr) {
    ESP_LOGE(TAG, "esp_websocket_client_init failed");
    this->schedule_reconnect_();
    return;
  }
  this->ws_handle_ = handle;

  esp_err_t err = esp_websocket_register_events(handle, WEBSOCKET_EVENT_ANY, va_ws_event_handler, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_websocket_register_events failed: %d", (int) err);
  }

  ESP_LOGI(TAG, "Connecting to %s", this->url_.c_str());
  err = esp_websocket_client_start(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_websocket_client_start failed: %d", (int) err);
    this->schedule_reconnect_();
  }
}

void VaClient::schedule_reconnect_() {
  // esp_websocket_client emits multiple events per failure (DISCONNECTED,
  // CLOSED, sometimes ERROR). Coalesce them into a single reconnect.
  if (this->reconnect_pending_) {
    return;
  }
  this->reconnect_pending_ = true;

  // One per *failure* (coalesced), not per individual WS event. Once we
  // cross the threshold fire the audible-error trigger exactly once until
  // a successful connect resets the counter.
  this->consecutive_failures_++;
  if (this->consecutive_failures_ >= kRepeatedFailureThreshold &&
      !this->repeated_failure_fired_) {
    this->repeated_failure_fired_ = true;
    ESP_LOGW(TAG, "%u consecutive reconnect failures — firing on_repeated_failure",
             (unsigned) this->consecutive_failures_);
    this->defer([this]() {
      for (auto *t : this->repeated_failure_triggers_) {
        t->trigger();
      }
    });
  }

  uint32_t delay = this->reconnect_delay_ms_;
  ESP_LOGI(TAG, "Scheduling reconnect in %u ms", (unsigned) delay);
  // Backoff schedule: 1s -> 2s -> 5s -> 10s (capped).
  if (this->reconnect_delay_ms_ < 2000) {
    this->reconnect_delay_ms_ = 2000;
  } else if (this->reconnect_delay_ms_ < 5000) {
    this->reconnect_delay_ms_ = 5000;
  } else {
    this->reconnect_delay_ms_ = 10000;
  }
  this->set_timeout("va_reconnect", delay, [this]() {
    this->reconnect_pending_ = false;
    this->connect_();
  });
}

void VaClient::on_ws_event(int32_t event_id, void *event_data) {
  auto *data = static_cast<esp_websocket_event_data_t *>(event_data);
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED: {
      ESP_LOGI(TAG, "WS connected");
      this->ws_connected_ = true;
      this->reconnect_delay_ms_ = 1000;  // reset backoff on a clean open
      // Don't reset the failure counter / fired flag yet — a flap-and-die
      // link would spam chimes. Only re-arm after the connection has held
      // for kStableConnectionMs without a disconnect.
      this->set_timeout("va_stable_connection", kStableConnectionMs, [this]() {
        if (this->ws_connected_) {
          this->consecutive_failures_ = 0;
          this->repeated_failure_fired_ = false;
          ESP_LOGD(TAG, "WS stable for %u ms — error chime re-armed",
                   (unsigned) kStableConnectionMs);
        }
      });

      const char start_msg[] = "{\"type\":\"start\"}";
      auto handle = static_cast<esp_websocket_client_handle_t>(this->ws_handle_);
      esp_websocket_client_send_text(handle, start_msg, sizeof(start_msg) - 1, portMAX_DELAY);
      this->apply_server_phase_("idle");
      break;
    }
    case WEBSOCKET_EVENT_DATA: {
      if (data == nullptr || data->data_ptr == nullptr || data->data_len <= 0)
        break;
      // op_code: 0x01 = text, 0x02 = binary, 0x00 = continuation of the prior
      // frame. esp_websocket_client splits long messages, so we must track the
      // type from the first chunk and feed continuations to the same handler.
      uint8_t op = data->op_code;
      if (op == 0x01) {
        this->last_data_was_binary_ = false;
        this->handle_text_(data->data_ptr, static_cast<size_t>(data->data_len));
      } else if (op == 0x02) {
        this->last_data_was_binary_ = true;
        this->handle_binary_(reinterpret_cast<const uint8_t *>(data->data_ptr),
                             static_cast<size_t>(data->data_len));
      } else if (op == 0x00) {
        // Continuation. Route based on the type of the in-flight message.
        if (this->last_data_was_binary_) {
          this->handle_binary_(reinterpret_cast<const uint8_t *>(data->data_ptr),
                               static_cast<size_t>(data->data_len));
        } else {
          this->handle_text_(data->data_ptr, static_cast<size_t>(data->data_len));
        }
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
      // Connection broke before the stability window elapsed — keep the
      // failure counter and the fired flag. A flapping link won't earn
      // a fresh chime.
      this->cancel_timeout("va_stable_connection");
      this->apply_server_phase_("idle");
      this->schedule_reconnect_();
      break;
    }
    default:
      break;
  }
}

void VaClient::handle_text_(const char *data, size_t len) {
  ESP_LOGD(TAG, "WS text: %.*s", (int) len, data);

  // ArduinoJson 7. The control channel sends tiny JSON objects — 128 bytes
  // of stack-backed JsonDocument is plenty for the largest message we send
  // today ({"type":"phase","value":"listening"} ≈ 38 bytes) with headroom
  // for future fields. If a message ever overflows, deserializeJson() returns
  // NoMemory and we fall through to the "unknown" branch below, which logs
  // and ignores — strictly safer than the previous substring scan which
  // could match across keys.
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    ESP_LOGW(TAG, "bad WS text frame: %s (%.*s)", err.c_str(), (int) len, data);
    return;
  }

  const char *type = doc["type"].as<const char *>();
  if (type == nullptr) {
    ESP_LOGW(TAG, "WS text frame missing 'type': %.*s", (int) len, data);
    return;
  }

  if (std::strcmp(type, "error") == 0) {
    const char *server_msg = doc["message"].as<const char *>();
    ESP_LOGW(TAG, "server error: %s", server_msg ? server_msg : "<no message>");
    // Without an audible cue the user just sees the LED go idle and
    // assumes the assistant ignored them. Reuse the on_repeated_failure
    // trigger — it already plays error_cloud_expired and the failure
    // mode is identical from the user's perspective ("something went
    // wrong, try again"). We deliberately don't bump consecutive_failures_
    // here; that counter is for WS reachability, not server-side errors.
    for (auto *t : this->repeated_failure_triggers_) {
      t->trigger();
    }
    this->apply_server_phase_("idle");
    return;
  }

  if (std::strcmp(type, "request_follow_up") == 0) {
    // Server's model called the request_follow_up tool — it asked a
    // question and wants the user to answer without saying a wake word.
    // Latch the modifier; the upcoming phase=idle will route us through
    // WaitingDrain → finish_drain_() which fires on_followup_opened
    // after the speaker chain empties. Only meaningful while a turn is
    // actually in flight; outside of that the next idle isn't a turn-
    // end signal anyway and the flag is harmless.
    ESP_LOGI(TAG, "request_follow_up received (state=%d, fill=%u bytes)",
             (int) this->current_state_, (unsigned) this->audio_fill_);
    this->request_follow_up_for_next_turn_ = true;
    return;
  }

  if (std::strcmp(type, "phase") == 0) {
    const char *value = doc["value"].as<const char *>();
    if (value == nullptr) {
      ESP_LOGW(TAG, "phase frame missing 'value': %.*s", (int) len, data);
      return;
    }
    // Whitelist the known phases — anything else is forward-compat noise.
    if (std::strcmp(value, "idle") == 0 || std::strcmp(value, "listening") == 0 ||
        std::strcmp(value, "thinking") == 0 || std::strcmp(value, "replying") == 0) {
      this->apply_server_phase_(value);
    } else {
      ESP_LOGD(TAG, "ignoring unknown phase '%s'", value);
    }
    return;
  }

  // hello / pong / anything we don't model yet — silently ignore.
  ESP_LOGD(TAG, "WS text ignored: type=%s", type);
}

void VaClient::handle_binary_(const uint8_t *data, size_t len) {
  if (this->speaker_ == nullptr || len < 2 || this->audio_buf_ == nullptr)
    return;
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
    // Reuse mono_buf_ as a scratch — it's already int16_t.
    this->mono_buf_.resize(pairs);
    float vol = this->volume_;
    if (vol < 0.0f) vol = 0.0f;
    else if (vol > 1.0f) vol = 1.0f;
    // Q15 fixed point so the inner loop stays integer-only.
    int32_t scale = static_cast<int32_t>(vol * 32768.0f);
#ifdef USE_VA_CLIENT_DIAGNOSTICS
    uint32_t clipped = 0;
#endif
    for (size_t i = 0; i < pairs; i++) {
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
      this->mono_buf_[i] = static_cast<int16_t>(v);
    }
#ifdef USE_VA_CLIENT_DIAGNOSTICS
    this->clipped_samples_ += clipped;
#endif
    data = reinterpret_cast<const uint8_t *>(this->mono_buf_.data());
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
  size_t tail = this->audio_tail_;
  size_t first = std::min(len, kAudioBufBytes - tail);
  std::memcpy(this->audio_buf_ + tail, data, first);
  if (first < len) {
    std::memcpy(this->audio_buf_, data + first, len - first);
  }
  this->audio_tail_ = (tail + len) % kAudioBufBytes;
  this->audio_fill_ += len;
  portEXIT_CRITICAL(&this->ring_mux_);
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
  // "phase":"idle" coming back from the server (response.done).
  if (!this->is_mic_streaming_())
    return;
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

  auto handle = static_cast<esp_websocket_client_handle_t>(this->ws_handle_);
  // 10ms timeout (~portTICK_PERIOD_MS): if WS task is briefly busy we wait
  // a tick rather than dropping the frame and spamming "Could not lock"
  // errors. If we're swamped, we accept dropping rather than blocking mic.
  esp_websocket_client_send_bin(handle, reinterpret_cast<const char *>(this->mono_buf_.data()),
                                static_cast<int>(this->mono_buf_.size() * sizeof(int16_t)),
                                10 / portTICK_PERIOD_MS);
}

// ---- State machine internals -----------------------------------------------

bool VaClient::is_mic_streaming_() const {
  // Single source of truth for whether on_mic_data_ should forward to the
  // WS. Listening covers both the pre-VAD window (after wake word, before
  // server speech_started) and the active turn. FollowupArmed runs briefly
  // between commit_followup_mic() and the next server phase=listening.
  return this->current_state_ == State::Listening ||
         this->current_state_ == State::FollowupArmed;
}

void VaClient::emit_phase_(const std::string &phase) {
  // transition_/emit_phase_ may run on the WS task; ESPHome triggers and
  // most component APIs are not thread-safe. Marshal the trigger fire
  // onto the main loop via defer(). We do NOT call speaker->stop() on
  // phase changes — the speaker task runs continuously after setup() and
  // play() just appends to its ring; stop/start churn was the root of an
  // earlier "Parent bus is busy" race.
  this->current_phase_ = phase;
  std::string phase_copy = phase;
  this->defer([this, phase_copy]() {
    for (auto *t : this->phase_triggers_) {
      t->trigger(phase_copy);
    }
  });
}

void VaClient::transition_(State next, const std::string &phase_label) {
  // One canonical helper for changing state + emitting a phase. Logs at
  // INFO when the state actually changes so the device log shows the full
  // lifecycle of every turn; bare LED re-renders (e.g. yaml-driven repaint
  // requests) go through emit_phase_ directly without churning the state.
  if (this->current_state_ != next) {
    ESP_LOGI(TAG, "state %d -> %d (%s)", (int) this->current_state_, (int) next,
             phase_label.c_str());
    this->current_state_ = next;
    this->state_entered_ms_ = millis();
  }
  this->emit_phase_(phase_label);
}

void VaClient::apply_server_phase_(const std::string &phase) {
  // Server-driven phase changes. The bridge is authoritative on lifecycle
  // (it sees the OpenAI events), so we generally trust whatever it sends.
  // The one nuance is phase=idle: it can land while we still have seconds
  // of TTS queued in PSRAM + downstream rings, in which case we defer the
  // LED-idle emit and the follow-up decision until finish_drain_() fires.
  ESP_LOGD(TAG, "server phase -> %s (state=%d)", phase.c_str(), (int) this->current_state_);

  if (phase == "listening") {
#ifdef USE_VA_CLIENT_DIAGNOSTICS
    if (this->turn_t_listening_ == 0 && this->turn_t_wake_ != 0) {
      this->turn_t_listening_ = millis();
    }
#endif
    // Server heard us — watchdog no longer needed.
    this->cancel_timeout("va_no_speech");
    this->cancel_timeout("va_followup");
    this->transition_(State::Listening, "listening");
    return;
  }

  if (phase == "thinking" || phase == "replying") {
#ifdef USE_VA_CLIENT_DIAGNOSTICS
    if (phase == "thinking" && this->turn_t_thinking_ == 0 && this->turn_t_wake_ != 0) {
      this->turn_t_thinking_ = millis();
    }
#endif
    // Either of these is a real turn in progress; cancel anything
    // related to draining or follow-up from a prior turn.
    this->cancel_timeout("va_followup");
    this->cancel_timeout("va_followup_open");
    this->cancel_timeout("va_tts_tail");
    this->cancel_timeout("va_no_speech");
    this->request_follow_up_for_next_turn_ = false;
    this->transition_(phase == "thinking" ? State::Thinking : State::Replying, phase);
    return;
  }

  if (phase == "idle") {
    // The interesting case. The state we're coming FROM dictates what
    // "idle" means.
    const State from = this->current_state_;
    const bool turn_just_ended = (from == State::Thinking || from == State::Replying);

    if (!turn_just_ended) {
      // Spurious idle from outside a turn (initial hello, post-disconnect).
      // Just sync the LED — no drain wait, no follow-up consideration.
      this->transition_(State::Idle, "idle");
      return;
    }

    if (this->interrupt_pending_) {
      // User barge-cancelled. Clean close, no drain wait (send_interrupt
      // already flushed the ring), no follow-up.
      this->interrupt_pending_ = false;
      this->request_follow_up_for_next_turn_ = false;
      this->cancel_timeout("va_tts_tail");
      this->transition_(State::Idle, "idle");
      return;
    }

    // Real end of turn. If everything has already played out we can go
    // straight to the post-turn decision (open follow-up vs Idle).
    // Otherwise enter WaitingDrain and let loop() drive finish_drain_()
    // when the speaker chain actually empties.
    if (this->audio_fill_ == 0) {
      this->current_state_ = State::WaitingDrain;
      this->state_entered_ms_ = millis();
      this->finish_drain_();
      return;
    }
    ESP_LOGI(TAG, "phase=idle but %u bytes still queued; LED + follow-up deferred",
             (unsigned) this->audio_fill_);
    this->current_state_ = State::WaitingDrain;
    this->state_entered_ms_ = millis();
    // No LED emit here — finish_drain_() will fire it once the chain drains.
    return;
  }

  ESP_LOGD(TAG, "ignoring unknown server phase '%s'", phase.c_str());
}

void VaClient::finish_drain_() {
  // Called from loop() once the PSRAM ring AND the downstream speaker
  // chain have both drained (or kSpeakerStopTimeoutMs elapsed). This is
  // the post-turn decision point: emit the deferred LED-idle, then either
  // open a follow-up mic window (request_follow_up case or kFollowupMs > 0)
  // or go straight to Idle.
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

  if (this->request_follow_up_for_next_turn_) {
    // Server explicitly asked us to keep the mic open for an answer.
    // yaml plays the chime via on_followup_opened, then calls
    // commit_followup_mic() once the chime + i2s tail is fully out.
    this->request_follow_up_for_next_turn_ = false;
    this->transition_(State::FollowupArmed, "idle");
    ESP_LOGI(TAG, "follow-up requested — firing on_followup_opened");
    this->defer([this]() {
      for (auto *t : this->followup_opened_triggers_) {
        t->trigger();
      }
    });
    return;
  }

  if (kFollowupMs > 0) {
    // Implicit follow-up window after every reply (XMOS AEC permitting).
    // Currently kFollowupMs is 0 so this branch never runs — kept so a
    // future re-enable is one constant flip rather than a state-machine
    // rewrite.
    this->transition_(State::FollowupArmed, "listening");
    ESP_LOGI(TAG, "implicit follow-up window open (%u ms)", (unsigned) kFollowupMs);
    this->set_timeout("va_followup", kFollowupMs, [this]() {
      if (this->current_state_ == State::FollowupArmed) {
        ESP_LOGI(TAG, "follow-up window expired");
        this->transition_(State::Idle, "idle");
      }
    });
    return;
  }

  // Default path: no follow-up. Close the turn cleanly.
  this->transition_(State::Idle, "idle");
}

void VaClient::start_session() {
  // Wake-word handler in yaml routes here. Open the mic so on_mic_data_
  // starts forwarding to the server. Without this gate, OpenAI Realtime's
  // server VAD would respond to any speech in the room and the wake word
  // would be cosmetic.

  // Belt-and-suspenders barge-in. The yaml wake handler also calls
  // send_interrupt() when it observes voice_assistant_phase == replying,
  // but two windows slip past that check:
  //   1) server already sent phase=idle yet PSRAM still has TTS queued
  //      (state == WaitingDrain). yaml's voice_assistant_phase has been
  //      reset to idle and the wake handler takes the "fresh session"
  //      path — no interrupt — so the new reply overlaps with the tail
  //      of the old.
  //   2) wake fires mid-reply on a long answer where the server is still
  //      generating tokens; without an interrupt, OpenAI keeps streaming
  //      TTS we'll never play, burning tokens.
  // The bridge treats interrupt as cheap when there's nothing to cancel
  // (response_cancel_not_active is in its benignCodes set), and
  // input_audio_buffer.clear is safe here because mic frames for the new
  // turn don't start flowing until after this function returns.
  const bool residual_reply =
      this->audio_fill_ > 0 ||
      this->current_state_ == State::Thinking ||
      this->current_state_ == State::Replying ||
      this->current_state_ == State::WaitingDrain;
  if (residual_reply) {
    ESP_LOGI(TAG, "start_session: interrupting residual reply (state=%d, fill=%u)",
             (int) this->current_state_, (unsigned) this->audio_fill_);
    this->send_interrupt();
  }

  // Wake starts a fresh session — drop any pending modifier flags from
  // the previous turn. State transition takes us to Listening; the
  // pending side-channels are reset here so the next phase=idle from the
  // server is interpreted as a fresh end-of-turn rather than a delayed
  // signal from the old one.
  this->request_follow_up_for_next_turn_ = false;
  this->interrupt_pending_ = false;
  this->cancel_timeout("va_followup");
  this->cancel_timeout("va_followup_open");
  this->cancel_timeout("va_tts_tail");
#ifdef USE_VA_CLIENT_DIAGNOSTICS
  // Anchor turn-latency timestamps for the new turn.
  this->turn_t_wake_ = millis();
  this->turn_t_listening_ = 0;
  this->turn_t_thinking_ = 0;
  this->turn_t_first_audio_out_ = 0;
  // Reset audio-quality detectors for this turn.
  this->last_binary_ms_ = 0;
  this->ws_gap_count_ = 0;
  this->ws_gap_max_ms_ = 0;
  this->clipped_samples_ = 0;
  this->underrun_logged_this_turn_ = false;
#endif
  this->transition_(State::Listening, "listening");
  // Watchdog: if server doesn't hear us within kNoSpeechTimeoutMs, abort
  // the session so we're not stuck with the mic open after a misfire.
  this->set_timeout("va_no_speech", kNoSpeechTimeoutMs, [this]() {
    if (this->current_state_ != State::Listening) {
      return;  // turn already progressed past Listening; nothing to do
    }
    ESP_LOGI(TAG, "no speech detected for %u ms — aborting session",
             (unsigned) kNoSpeechTimeoutMs);
    if (this->ws_connected_ && this->ws_handle_ != nullptr) {
      const char m[] = "{\"type\":\"interrupt\"}";
      auto handle = static_cast<esp_websocket_client_handle_t>(this->ws_handle_);
      esp_websocket_client_send_text(handle, m, sizeof(m) - 1, portMAX_DELAY);
    }
#ifdef USE_VA_CLIENT_DIAGNOSTICS
    this->turn_t_wake_ = 0;
#endif
    this->transition_(State::Idle, "idle");
  });
}

void VaClient::commit_followup_mic() {
  // Called from yaml's on_followup_opened automation once the chime has
  // finished playing AND the i2s tail has cleared (wait_until + delay).
  // If anything pre-empted us between trigger fire and here (a fresh
  // wake word, a Stop, send_interrupt, or a new turn starting) the state
  // already moved out of FollowupArmed — silently no-op so we don't
  // reopen the mic out of nowhere.
  if (this->current_state_ != State::FollowupArmed) {
    ESP_LOGD(TAG, "commit_followup_mic: state=%d, ignoring",
             (int) this->current_state_);
    return;
  }
  ESP_LOGI(TAG, "follow-up mic armed by yaml (window %u ms)",
           (unsigned) kRequestFollowUpMs);
  this->transition_(State::Listening, "listening");
  this->set_timeout("va_followup", kRequestFollowUpMs, [this]() {
    if (this->current_state_ == State::Listening) {
      ESP_LOGI(TAG, "follow-up window expired");
      this->transition_(State::Idle, "idle");
    }
  });
}

void VaClient::send_interrupt() {
  if (!this->ws_connected_ || this->ws_handle_ == nullptr) {
    ESP_LOGW(TAG, "send_interrupt: WS not connected");
    return;
  }
  const char msg[] = "{\"type\":\"interrupt\"}";
  auto handle = static_cast<esp_websocket_client_handle_t>(this->ws_handle_);
  esp_websocket_client_send_text(handle, msg, sizeof(msg) - 1, portMAX_DELAY);
  // Flush our PSRAM playback queue — what's already been pushed into the
  // resampler/mixer/leaf will still drain (~600 ms residual), but everything
  // we have yet to hand off is dropped. The yaml side stops the resampler
  // explicitly. The ring reset has to happen under the mux: the WS task
  // could be mid-write and seeing head=tail=fill=0 partway through would
  // let it write into a "freshly empty" buffer the user just barge-
  // cancelled.
  portENTER_CRITICAL(&this->ring_mux_);
  this->audio_head_ = 0;
  this->audio_tail_ = 0;
  this->audio_fill_ = 0;
  portEXIT_CRITICAL(&this->ring_mux_);
  this->request_follow_up_for_next_turn_ = false;
  this->cancel_timeout("va_no_speech");
  this->cancel_timeout("va_followup");
  this->cancel_timeout("va_followup_open");
  this->cancel_timeout("va_tts_tail");
  // The phase=idle the server is about to send shouldn't open a follow-
  // up mic window — user said "stop", not "wait for me to keep talking".
  // apply_server_phase_("idle") consumes this on the next idle.
  this->interrupt_pending_ = true;
  ESP_LOGI(TAG, "send_interrupt — WS msg sent, queue flushed");
}

}  // namespace va_client
}  // namespace esphome
