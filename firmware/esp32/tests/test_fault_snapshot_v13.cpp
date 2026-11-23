#include "../traction_control_v4_2_uart_shadow_v13_brake_continuity/fault_snapshot.h"

#include <cassert>
#include <cstdio>

int main()
{
    using Decision = FaultCaptureDecision;

    // No active command: diagnostic capture is forbidden.
    assert(decideFaultCapture(
        false, false, false, false, false
    ) == Decision::NO_ACTIVE_COMMAND);

    assert(decideFaultCapture(
        false, true, true, true, false
    ) == Decision::NO_ACTIVE_COMMAND);

    // First FAULT of an active command.
    assert(decideFaultCapture(
        true, false, false, false, false
    ) == Decision::CAPTURE_NEW_FAULT);

    // Repeated fault trigger for the same command.
    assert(decideFaultCapture(
        true, true, true, true, false
    ) == Decision::PRESERVE_FIRST_TRIGGER);

    // Different command in the same boot session.
    assert(decideFaultCapture(
        true, true, true, false, false
    ) == Decision::CAPTURE_NEW_FAULT);

    // Same command ID in another boot session.
    assert(decideFaultCapture(
        true, true, false, true, false
    ) == Decision::CAPTURE_NEW_FAULT);

    // Different command and boot session.
    assert(decideFaultCapture(
        true, true, false, false, false
    ) == Decision::CAPTURE_NEW_FAULT);

    // ACK_FAULT opens a new episode, even with the same command ID.
    assert(decideFaultCapture(
        true, true, true, true, true
    ) == Decision::CAPTURE_NEW_FAULT);

    // Once captured, subsequent triggers preserve that episode.
    assert(decideFaultCapture(
        true, true, true, true, false
    ) == Decision::PRESERVE_FIRST_TRIGGER);

    // An episode cannot be captured without an active command.
    assert(decideFaultCapture(
        false, true, true, true, true
    ) == Decision::NO_ACTIVE_COMMAND);

    std::puts("FAULT_SNAPSHOT_POLICY: PASS");
    return 0;
}
