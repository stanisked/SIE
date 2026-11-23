#pragma once

#include <stdint.h>

// Diagnostic-only trace. All times are monotonic 32-bit micros() samples.
// The intervals are short and unsigned subtraction is rollover-safe.
// Encoder pairs are individually sampled, not synchronously latched.
struct ShadowTurnBrakeTrace {
  uint32_t requestSampledUs = 0;
  uint32_t mutexAcquiredUs = 0;
  uint32_t outputCommandsIssuedUs = 0;
  int32_t rightCountAtOutputCommands = 0;
  int32_t leftCountAtOutputCommands = 0;
};

inline uint32_t brakeTimingElapsedUs(uint32_t start, uint32_t end)
{
  return end - start;
}
