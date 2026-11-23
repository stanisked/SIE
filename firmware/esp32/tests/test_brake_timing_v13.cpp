#include "../traction_control_v4_2_uart_shadow_v13_brake_continuity/brake_timing.h"

#include <cassert>
#include <cstdint>
#include <cstdio>

int main()
{
  assert(brakeTimingElapsedUs(100, 125) == 25);
  assert(brakeTimingElapsedUs(0xFFFFFFFAu, 5u) == 11u);
  ShadowTurnBrakeTrace trace{};
  trace.requestSampledUs = 1000;
  trace.mutexAcquiredUs = 1090;
  trace.outputCommandsIssuedUs = 1120;
  trace.rightCountAtOutputCommands = 18;
  trace.leftCountAtOutputCommands = 17;
  assert(brakeTimingElapsedUs(
      trace.requestSampledUs, trace.outputCommandsIssuedUs) == 120u);
  assert(brakeTimingElapsedUs(
      trace.requestSampledUs, trace.mutexAcquiredUs) == 90u);
  assert(brakeTimingElapsedUs(
      trace.mutexAcquiredUs, trace.outputCommandsIssuedUs) == 30u);
  assert(trace.rightCountAtOutputCommands == 18);
  assert(trace.leftCountAtOutputCommands == 17);
  std::puts("BRAKE_TIMING_MATH: PASS");
  return 0;
}
