#include "udp_log.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/logger/logger.h"
#include "esphome/components/network/util.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/time.h"

#include <cstdio>
#include <cstring>

#include <lwip/netdb.h>

namespace esphome {
namespace udp_log {

static const char *const TAG = "udp_log";

void UdpLog::setup() {
  this->sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->sock_fd_ < 0) {
    ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
    return;
  }
  // Non-blocking: a slow/unreachable peer must not stall the logger task.
  int flags = fcntl(this->sock_fd_, F_GETFL, 0);
  fcntl(this->sock_fd_, F_SETFL, flags | O_NONBLOCK);

  this->dest_.sin_family = AF_INET;
  this->dest_.sin_port = htons(this->port_);

  // DO NOT call getaddrinfo() here — DNS isn't up yet during setup() and
  // a blocking lookup against va.local easily hangs >10 s, tripping the
  // task watchdog. We resolve lazily from the periodic retry below, but
  // only when the network is up.
  //
  // 5-second tick, plenty for an idle resolve attempt; stops itself once
  // resolved successfully.
  this->set_interval("udp_log_resolve", 5000, [this]() {
    if (this->resolved_)
      return;
    if (!network::is_connected())
      return;
    if (this->resolve_()) {
      this->cancel_interval("udp_log_resolve");
    }
  });

  // Register with ESPHome's logger callbacks. The real implementation is
  // only compiled in when USE_LOG_LISTENERS is defined — we trigger that
  // via logger.request_log_listener() in our __init__.py codegen.
  logger::global_logger->add_log_callback(this, &UdpLog::log_trampoline_);

  ESP_LOGCONFIG(TAG, "UDP log -> %s:%u (level>=%d)", this->host_.c_str(),
                (unsigned) this->port_, this->min_level_);
}

void UdpLog::dump_config() {
  ESP_LOGCONFIG(TAG, "UDP Log:");
  ESP_LOGCONFIG(TAG, "  Host: %s", this->host_.c_str());
  ESP_LOGCONFIG(TAG, "  Port: %u", (unsigned) this->port_);
  ESP_LOGCONFIG(TAG, "  Min level: %d", this->min_level_);
  ESP_LOGCONFIG(TAG, "  Resolved: %s", this->resolved_ ? "yes" : "no");
}

bool UdpLog::resolve_() {
  if (this->resolved_)
    return true;
  struct addrinfo hints = {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  struct addrinfo *res = nullptr;
  int rc = ::getaddrinfo(this->host_.c_str(), nullptr, &hints, &res);
  if (rc != 0 || res == nullptr) {
    ESP_LOGD(TAG, "resolve %s: failed rc=%d", this->host_.c_str(), rc);
    return false;
  }
  auto *sin = reinterpret_cast<struct sockaddr_in *>(res->ai_addr);
  this->dest_.sin_addr = sin->sin_addr;
  char ip[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &this->dest_.sin_addr, ip, sizeof(ip));
  ::freeaddrinfo(res);
  // IMPORTANT: set resolved_ BEFORE the ESP_LOGI below. ESP_LOG fires our
  // own log callback synchronously; if resolved_ is still false the first
  // log line — the one announcing success — gets dropped at the on_log_
  // guard. Flip the flag first so the announcement reaches the sink too.
  this->resolved_ = true;
  ESP_LOGI(TAG, "resolved %s -> %s:%u", this->host_.c_str(), ip, (unsigned) this->port_);
  return true;
}

void UdpLog::log_trampoline_(void *self_v, uint8_t level, const char *tag,
                             const char *message, size_t length) {
  auto *self = static_cast<UdpLog *>(self_v);
  if (self != nullptr)
    self->on_log_(level, tag, message, length);
}

// ESPHome → Dozzle level name mapping. Dozzle's level_guesser checks the
// JSON `level` field but only accepts STRING values ("error", "warn",
// "info", "debug", "trace"); pino-style integers fall through unmatched.
// We emit strings so Dozzle paints severity dots correctly.
static const char *esphome_level_to_name_(int level) {
  switch (level) {
    case 1:  // ESPHOME_LOG_LEVEL_ERROR
      return "error";
    case 2:  // ESPHOME_LOG_LEVEL_WARN
      return "warn";
    case 3:  // ESPHOME_LOG_LEVEL_INFO
    case 4:  // ESPHOME_LOG_LEVEL_CONFIG
      return "info";
    case 5:  // ESPHOME_LOG_LEVEL_DEBUG
      return "debug";
    default:  // VERBOSE / VERY_VERBOSE
      return "trace";
  }
}

// Append `src[0..len)` to `buf[*pos..cap)`, JSON-escaping double quotes,
// backslashes, common whitespace, control chars, and ANSI escape
// sequences ("\x1b[...m") which ESPHome's log message can contain when
// colours are enabled. Stops early if there's no room; we never overflow
// `buf` and always leave space for the trailing `"}\n`.
static void json_escape_(const char *src, size_t len, char *buf, size_t cap, size_t *pos) {
  size_t p = *pos;
  for (size_t i = 0; i < len; i++) {
    if (p + 8 >= cap)
      break;
    unsigned char c = static_cast<unsigned char>(src[i]);
    // Strip ANSI CSI sequences inline so msg stays readable in JSON.
    if (c == 0x1b && i + 1 < len && src[i + 1] == '[') {
      i += 2;
      while (i < len && !((src[i] >= 'A' && src[i] <= 'Z') ||
                          (src[i] >= 'a' && src[i] <= 'z'))) {
        i++;
      }
      continue;  // skip the final letter too (for-loop's ++i)
    }
    switch (c) {
      case '"':
      case '\\':
        buf[p++] = '\\';
        buf[p++] = c;
        break;
      case '\n':
        buf[p++] = '\\';
        buf[p++] = 'n';
        break;
      case '\r':
        buf[p++] = '\\';
        buf[p++] = 'r';
        break;
      case '\t':
        buf[p++] = '\\';
        buf[p++] = 't';
        break;
      default:
        if (c < 0x20) {
          int n = std::snprintf(buf + p, cap - p, "\\u%04x", c);
          if (n > 0)
            p += static_cast<size_t>(n);
        } else {
          buf[p++] = static_cast<char>(c);
        }
        break;
    }
  }
  *pos = p;
}

void UdpLog::on_log_(int level, const char *tag, const char *message, size_t length) {
  if (level > this->min_level_)
    return;
  if (this->sending_)
    return;  // reentrancy guard
  if (this->sock_fd_ < 0 || !this->resolved_)
    return;
  // Skip our own logs to avoid feedback loops if sendto() ever logs an error.
  if (tag != nullptr && std::strcmp(tag, TAG) == 0)
    return;

  this->sending_ = true;
  // Emit pino-compatible JSON so Dozzle renders these alongside the
  // voice-assistant container's output with the same colouring / level
  // chips. The `message` ESPHome hands us is the full formatted line
  // ("[hh:mm:ss][I][tag:line]: text"); ship it as-is in `msg` to preserve
  // source line info. `scope` carries the bare tag for filtering.
  // No `time` field — Dozzle stamps each packet from the docker side.
  char buf[640];
  size_t p = 0;
  int n = std::snprintf(buf, sizeof(buf), "{\"level\":\"%s\"",
                        esphome_level_to_name_(level));
  if (n > 0)
    p = static_cast<size_t>(n);
  // Device name (App.get_name(), MAC-suffixed) so several speakers sharing
  // one UDP sink stay distinguishable in Dozzle — ESPHome's formatted line
  // carries no device identifier. Emitted as a self-closed JSON field.
  if (p + 12 < sizeof(buf)) {
    std::memcpy(buf + p, ",\"device\":\"", 11);
    p += 11;
  }
  const std::string &device = App.get_name();
  json_escape_(device.c_str(), device.size(), buf, sizeof(buf), &p);
  if (p + 1 < sizeof(buf))
    buf[p++] = '"';
  // pino-style time: epoch millis. Only emit if NTP has synced (timestamp
  // > 2020-01-01 sanity check); otherwise the reading is meaningless and
  // would confuse log viewers. millis()-since-boot can be inferred from
  // packet arrival if needed.
  if (this->time_ != nullptr) {
    auto now = this->time_->now();
    if (now.is_valid() && now.timestamp > 1577836800 /* 2020-01-01 */) {
      uint64_t ts_ms =
          static_cast<uint64_t>(now.timestamp) * 1000ULL + (millis() % 1000ULL);
      n = std::snprintf(buf + p, sizeof(buf) - p, ",\"time\":%llu",
                        static_cast<unsigned long long>(ts_ms));
      if (n > 0)
        p += static_cast<size_t>(n);
    }
  }
  if (p + 11 < sizeof(buf)) {
    std::memcpy(buf + p, ",\"scope\":\"", 10);
    p += 10;
  }
  if (tag != nullptr)
    json_escape_(tag, std::strlen(tag), buf, sizeof(buf), &p);
  if (p + 9 < sizeof(buf)) {
    std::memcpy(buf + p, "\",\"msg\":\"", 9);
    p += 9;
  }
  json_escape_(message, length, buf, sizeof(buf), &p);
  // Always leave room for the closing `"}\n`.
  if (p + 3 > sizeof(buf))
    p = sizeof(buf) - 3;
  buf[p++] = '"';
  buf[p++] = '}';
  buf[p++] = '\n';

  ::sendto(this->sock_fd_, buf, p, 0, reinterpret_cast<const struct sockaddr *>(&this->dest_),
           sizeof(this->dest_));
  this->sending_ = false;
}

}  // namespace udp_log
}  // namespace esphome
