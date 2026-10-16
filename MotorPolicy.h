#pragma once
#include <stdint.h>

// Hardware-independent policy shared by the sketch and host behavior tests.
namespace MotorPolicy {
inline int bounded(int value, int low, int high, int fallback) {
  return value >= low && value <= high ? value : fallback;
}
struct Settings { int forward; int reverse; int pulseMs; };
template<class Preferences> Settings load(Preferences& prefs) {
  Settings s;
  s.forward = bounded(prefs.getInt("speed", 60), 20, 100, 60);
  s.reverse = bounded(prefs.getInt("rev_spd", s.forward), 20, 100, s.forward);
  // Snapshot the legacy speed once, so later forward changes cannot alter reverse on reboot.
  if (!prefs.isKey("rev_spd")) prefs.putInt("rev_spd", s.reverse);
  s.pulseMs = bounded(prefs.getInt("s_pulse", 100), 20, 500, 100);
  return s;
}
// Reject malformed/overflowing/out-of-range input without changing the output.
inline bool parseSetting(const char* text, int low, int high, int& output) {
  if (!text || !*text) return false;
  int value = 0;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9') return false;
    int digit = *text - '0';
    if (value > (high - digit) / 10) return false;
    value = value * 10 + digit;
  }
  if (value < low || value > high) return false;
  output = value;
  return true;
}
// A timeout/stop never rearms a held input. Only an explicit neutral/release does.
class SteeringPulse {
  bool held_ = false;
  bool active_ = false;
  uint32_t start_ = 0;
  uint32_t duration_ = 100;
public:
  bool begin(uint32_t now, int durationMs) {
    if (held_) return false;
    held_ = true;
    active_ = true;
    start_ = now;
    duration_ = bounded(durationMs, 20, 500, 100);
    return true;
  }
  bool expired(uint32_t now) const {
    return active_ && uint32_t(now - start_) >= duration_;
  }
  void stop() { active_ = false; }
  void release() { stop(); held_ = false; }
};
} // namespace MotorPolicy
