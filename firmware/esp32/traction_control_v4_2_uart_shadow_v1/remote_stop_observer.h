#pragma once
#include <stdint.h>

// Encoder quietness is an observation, not proof against wheel slip or sensor failure.
struct RemoteStopObserver {
  static constexpr uint32_t hold_ms = 400;
  static constexpr uint32_t sample_ms = 100;
  static constexpr uint32_t deadline_ms = 2000;
  bool pending = false;
  bool timed_out = false;
  bool complete = false;
  uint32_t started_ms = 0;
  uint32_t last_sample_ms = 0;
  uint32_t completed_ms = 0;
  int32_t start_right = 0, start_left = 0;
  int32_t final_right = 0, final_left = 0;
  uint8_t quiet_intervals = 0;

  void begin(uint32_t now, int32_t right, int32_t left) {
    if (pending) return; // repeated HTTP stop must not reset the clock
    *this = RemoteStopObserver{};
    pending = true;
    started_ms = last_sample_ms = now;
    start_right = final_right = right;
    start_left = final_left = left;
  }
  bool update(uint32_t now, int32_t right, int32_t left) {
    if (!pending) return false;
    const uint32_t elapsed = now - started_ms;
    if (elapsed >= deadline_ms) timed_out = true;
    if (elapsed < hold_ms || now - last_sample_ms < sample_ms) return false;
    // Two quiet intervals after the hold, separated by real elapsed time.
    if (last_sample_ms - started_ms < hold_ms) {
      quiet_intervals = 0;
    } else if (right == final_right && left == final_left) {
      ++quiet_intervals;
    } else {
      quiet_intervals = 0;
    }
    last_sample_ms = now;
    final_right = right;
    final_left = left;
    if (quiet_intervals < 2) return false;
    pending = false;
    complete = true;
    completed_ms = now;
    return true;
  }
};

