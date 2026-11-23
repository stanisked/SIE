#pragma once

// Pure capture policy for bounded-motion fault diagnostics.
// No Arduino dependencies, motor control or mutable global state.

enum class FaultCaptureDecision {
  NO_ACTIVE_COMMAND,
  PRESERVE_FIRST_TRIGGER,
  CAPTURE_NEW_FAULT
};

inline FaultCaptureDecision decideFaultCapture(
    bool activeCommandPresent,
    bool snapshotValid,
    bool sameBootSession,
    bool sameCommandId,
    bool newFaultEpisode)
{
  if (!activeCommandPresent) {
    return FaultCaptureDecision::NO_ACTIVE_COMMAND;
  }

  if (snapshotValid && sameBootSession && sameCommandId &&
      !newFaultEpisode) {
    return FaultCaptureDecision::PRESERVE_FIRST_TRIGGER;
  }

  return FaultCaptureDecision::CAPTURE_NEW_FAULT;
}
