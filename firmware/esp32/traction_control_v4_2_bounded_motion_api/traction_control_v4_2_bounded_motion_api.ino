#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_system.h>

/*
 * SIE mobile base traction and in-place turn controller v3 test
 *
 * Hardware validated on:
 *   ESP32 DevKit / ESP-WROOM-32
 *   ZK-5AD dual H-bridge
 *   2 x JGB37-520, 6 V, 200 rpm, quadrature encoders
 *   Wheel diameter: 67 mm
 *   Track width: 290 mm
 *
 * Safety model:
 *   - motor outputs are LOW during boot and Wi-Fi connection;
 *   - movement requires an explicit HTTP POST /start;
 *   - HTTP POST /stop cuts PWM immediately;
 *   - Wi-Fi loss, encoder loss, wrong direction, timeout, or distance-limit
 *     violation stops both motors;
 *   - target stopping uses a short TA6586 active-brake pulse, then coasting.
 *
 * GPIO26 is intentionally unused. The tested board did not pull GPIO26
 * reliably LOW, so ZK-5AD D1 was moved to GPIO33.
 *
 * Use only on a trusted private 2.4 GHz Wi-Fi network.
 */

// Copy the credentials from the validated Wi-Fi diagnostic sketch.
// Keep the password private; do not publish this source with real credentials.
#include "wifi_credentials.h"

constexpr char FIRMWARE_ID[] = "traction_control_v4_2_bounded_motion_api";

// ---------------------------------------------------------------------------
// ZK-5AD control pins
// ---------------------------------------------------------------------------

constexpr uint8_t ZK_D0 = 25;  // right wheel reverse
constexpr uint8_t ZK_D1 = 33;  // right wheel forward
constexpr uint8_t ZK_D2 = 27;  // left wheel reverse
constexpr uint8_t ZK_D3 = 14;  // left wheel forward

// ---------------------------------------------------------------------------
// Encoder pins
// ---------------------------------------------------------------------------

constexpr uint8_t RIGHT_A = 18;
constexpr uint8_t RIGHT_B = 19;
constexpr uint8_t LEFT_A = 21;
constexpr uint8_t LEFT_B = 22;

// ---------------------------------------------------------------------------
// Calibrated geometry and encoder scale
// ---------------------------------------------------------------------------

// Loaded effective diameters calibrated from a measured 0.500 m floor run.
// The unloaded physical tread diameter remains approximately 67 mm.
constexpr float RIGHT_WHEEL_DIAMETER_M = 0.0646f;
constexpr float LEFT_WHEEL_DIAMETER_M = 0.0652f;
constexpr float TRACK_WIDTH_M = 0.290f;
// Loaded square tests repeatedly under-rotated by about 5 degrees per 360.
// This effective turn track captures tire scrub and caster resistance.
constexpr float TURN_EFFECTIVE_TRACK_WIDTH_M = 0.2941f;

// One-channel RISING decoding, measured manually at the wheel output.
constexpr float RIGHT_COUNTS_PER_REV = 205.0f;
constexpr float LEFT_COUNTS_PER_REV = 206.0f;

constexpr float RIGHT_M_PER_COUNT =
    PI * RIGHT_WHEEL_DIAMETER_M / RIGHT_COUNTS_PER_REV;

constexpr float LEFT_M_PER_COUNT =
    PI * LEFT_WHEEL_DIAMETER_M / LEFT_COUNTS_PER_REV;

// ---------------------------------------------------------------------------
// First loaded motion profile
// ---------------------------------------------------------------------------

constexpr float FORWARD_TARGET_DISTANCE_M = 0.500f;
constexpr float REVERSE_TARGET_DISTANCE_M = 0.200f;
constexpr float TURN_TARGET_ANGLE_RAD = PI * 0.5f;
constexpr float TURN_SINGLE_TARGET_WHEEL_DISTANCE_M =
    TRACK_WIDTH_M * TURN_TARGET_ANGLE_RAD * 0.5f;
constexpr float TURN_SQUARE_TARGET_WHEEL_DISTANCE_M =
    TURN_EFFECTIVE_TRACK_WIDTH_M * TURN_TARGET_ANGLE_RAD * 0.5f;
constexpr float MAX_PROFILE_SPEED_M_S = 0.140f;
constexpr float TURN_MAX_PROFILE_SPEED_M_S = 0.090f;
constexpr float ACCEL_M_S2 = 0.200f;
constexpr float DECEL_M_S2 = 0.120f;

// Loaded tests fit a constant-deceleration model: d_stop = K * speed^2.
constexpr float BRAKE_DISTANCE_GAIN_S2_PER_M = 0.460f;
constexpr float MIN_STOP_MARGIN_M = 0.010f;
constexpr float MAX_STOP_MARGIN_M = 0.035f;
constexpr float TURN_MIN_STOP_MARGIN_M = 0.002f;
constexpr float TURN_MAX_STOP_MARGIN_M = 0.025f;

constexpr uint32_t CONTROL_PERIOD_MS = 100;
constexpr uint32_t FORWARD_TIMEOUT_MS = 12000;
constexpr uint32_t REVERSE_TIMEOUT_MS = 6000;
constexpr uint32_t TURN_TIMEOUT_MS = 7000;
constexpr float BOUNDED_FORWARD_MIN_DISTANCE_M = 0.020f;
constexpr float BOUNDED_FORWARD_MAX_DISTANCE_M = 0.100f;
constexpr float BOUNDED_TURN_MIN_ANGLE_DEG = 1.0f;
constexpr float BOUNDED_TURN_MAX_ANGLE_DEG = 10.0f;
constexpr uint32_t BOUNDED_FORWARD_TIMEOUT_MS = 5000;
constexpr uint32_t BOUNDED_TURN_TIMEOUT_MS = 4000;
constexpr uint32_t BOUNDED_ACTIVE_BRAKE_HOLD_MS = 400;
constexpr uint32_t BOUNDED_SETTLE_OBSERVE_MS = 100;
constexpr uint32_t BOUNDED_MICRO_TURN_CORRECTION_PULSE_MS = 100;
// The single-pulse synchronization challenge deliberately disables blind
// post-settle correction pulses.
constexpr uint8_t BOUNDED_MICRO_TURN_MAX_CORRECTION_PULSES = 0;
constexpr uint8_t BOUNDED_SETTLED_STABLE_SAMPLES = 2;
// Bounded turns are only 1..10 degrees. Their legacy 90-degree motion profile
// starts braking too late at this scale, so reserve encoder room explicitly.
constexpr int32_t BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS = 6;
constexpr uint8_t COMMAND_HISTORY_CAPACITY = 16;
constexpr uint32_t ACTIVE_BRAKE_TIME_MS = 100;
constexpr uint32_t COAST_TIME_MS = 500;
constexpr uint32_t SQUARE_SETTLE_TIME_MS = 1200;
constexpr uint32_t SQUARE_TOTAL_TIMEOUT_MS = 60000;
constexpr uint8_t SQUARE_SEGMENT_COUNT = 8;

// Abort if travel exceeds the selected target by this amount.
constexpr float HARD_DISTANCE_EXTRA_M = 0.150f;
constexpr float TURN_HARD_DISTANCE_EXTRA_M = 0.060f;

// ---------------------------------------------------------------------------
// Adaptive loaded start
// ---------------------------------------------------------------------------

// Measured loaded breakaway was 215-225. Ramp from below the threshold and
// stop raising an individual wheel as soon as continuous movement is proven.
constexpr int BREAKAWAY_START_PWM = 180;
constexpr int BREAKAWAY_MAX_PWM = 235;
constexpr int BREAKAWAY_PWM_STEP = 5;

constexpr uint32_t BREAKAWAY_STEP_MS = 20;
constexpr uint32_t BREAKAWAY_TIMEOUT_MS = 450;

// Three counts only selected gearbox lash. Fifteen counts proved translation.
constexpr int32_t BREAKAWAY_DETECT_COUNTS = 15;

// ---------------------------------------------------------------------------
// Loaded running model and PI correction
// ---------------------------------------------------------------------------

// At approximately 0.14 m/s on the finalized three-point chassis:
//   right PWM 170 -> 67 counts / 0.5 s
//   left  PWM 135 -> 69 counts / 0.5 s
constexpr int RIGHT_RUN_PWM_AT_MAX = 170;
constexpr int LEFT_RUN_PWM_AT_MAX = 135;

// In-place turns have higher scrub resistance than straight travel.
// Keep both wheels above this loaded sustain level only during turns.
constexpr int TURN_SUSTAIN_PWM = 180;

// Approximate rolling feed-forward at very low target speed.
constexpr int RIGHT_RUN_PWM_MIN = 115;
constexpr int LEFT_RUN_PWM_MIN = 95;

constexpr int MAX_DRIVE_PWM = 235;

// PI terms operate on wheel-speed error in m/s.
constexpr float KP_SPEED = 100.0f;
constexpr float KI_SPEED = 40.0f;
constexpr float INTEGRAL_LIMIT = 0.20f;

// If a wheel is above its target by this amount, remove drive and let it coast.
constexpr float OVERSPEED_COAST_MARGIN_M_S = 0.025f;

// Cross-wheel path synchronization.
constexpr float SYNC_GAIN = 1.5f;
constexpr float MAX_SYNC_CORRECTION_M_S = 0.025f;

// The traction floor can prevent the speed loop from reducing the leading
// wheel. Add PWM only to the lagging wheel so both sides retain enough torque.
constexpr float SYNC_PWM_GAIN_PER_M = 4000.0f;
constexpr int MAX_SYNC_PWM_BOOST = 60;

// Encoder/stall watchdog during the regulated phase.
constexpr float STALL_CHECK_MIN_TARGET_M_S = 0.060f;
constexpr uint8_t STALL_CYCLES_LIMIT = 3;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------

volatile int32_t rightRawCount = 0;
volatile int32_t leftRawCount = 0;

WebServer server(80);

enum class MotionState : uint8_t {
  READY,
  STARTING,
  DRIVING,
  BRAKING,
  CORRECTING,
  COASTING,
  SUCCESS,
  FAULT
};

enum class MotionDirection : uint8_t {
  FORWARD,
  REVERSE,
  TURN_LEFT,
  TURN_RIGHT
};

enum class SquareState : uint8_t {
  IDLE,
  RUNNING,
  SUCCESS,
  FAULT
};

enum class CommandState : uint8_t {
  ACCEPTED,
  STARTING,
  DRIVING,
  BRAKING,
  COASTING,
  SUCCESS,
  PARTIAL_PROGRESS,
  FAULT,
  STOPPED
};

enum class BoundedMotionKind : uint8_t {
  NONE,
  LINEAR,
  TURN
};

enum class BoundedMotionProfile : uint8_t {
  NONE,
  BOUNDED_FORWARD_V1,
  BOUNDED_MICRO_TURN_V1
};

enum class BoundedStopMode : uint8_t {
  NONE,
  ACTIVE_BRAKE,
  COAST
};

enum class BoundedForwardBrakeTrigger : uint8_t {
  NONE,
  PREDICTIVE_BRAKE,
  TARGET_REACHED,
  HARD_LIMIT
};

struct CommandRecord {
  bool used = false;
  String commandId;
  String endpoint;
  bool hasDistance = false;
  float distanceM = 0.0f;
  bool hasAngle = false;
  float angleDeg = 0.0f;
  BoundedMotionKind boundedKind = BoundedMotionKind::NONE;
  BoundedMotionProfile boundedMotionProfile = BoundedMotionProfile::NONE;
  int32_t turnBrakeReserveCounts = 0;
  bool rightIndividualBrakeStarted = false;
  bool leftIndividualBrakeStarted = false;
  int32_t rightCountAtIndividualBrake = 0;
  int32_t leftCountAtIndividualBrake = 0;
  int rightLastAppliedPwm = 0;
  int leftLastAppliedPwm = 0;
  int32_t syncErrorCounts = 0;
  float boundedActualProgressRatio = 0.0f;
  bool reobserveRequired = false;
  int32_t rightTargetCounts = 0;
  int32_t leftTargetCounts = 0;
  int32_t rightLimitCounts = 0;
  int32_t leftLimitCounts = 0;
  int32_t rightCompletionToleranceCounts = 0;
  int32_t leftCompletionToleranceCounts = 0;
  int32_t rightMinSuccessCounts = 0;
  int32_t leftMinSuccessCounts = 0;
  int32_t rightBrakeStartCounts = 0;
  int32_t leftBrakeStartCounts = 0;
  int32_t rightCountAtBrakeStart = 0;
  int32_t leftCountAtBrakeStart = 0;
  int32_t rightFinalCount = 0;
  int32_t leftFinalCount = 0;
  int32_t rightOvershootCounts = 0;
  int32_t leftOvershootCounts = 0;
  BoundedStopMode stopMode = BoundedStopMode::NONE;
  uint32_t boundedTimeoutMs = 0;
  bool boundedInitialPulseCompleted = false;
  uint8_t boundedCorrectionPulsesUsed = 0;
  uint8_t boundedMaxCorrectionPulses = 0;
  int32_t boundedCountsBeforeCorrectionRight = 0;
  int32_t boundedCountsBeforeCorrectionLeft = 0;
  float boundedTurnMinSuccessAngleRad = 0.0f;
  float boundedTurnFinalAngleRad = 0.0f;
  int32_t boundedTurnMinWheelProgressCounts = 0;
  bool forwardPreBrakeGuardCaptured = false;
  uint32_t forwardPreBrakeGuardTimestampMs = 0;
  MotionState forwardPreBrakeGuardState = MotionState::READY;
  int32_t forwardPreBrakeRightCount = 0;
  int32_t forwardPreBrakeLeftCount = 0;
  int32_t forwardPreBrakeRightBrakeStartCounts = 0;
  int32_t forwardPreBrakeLeftBrakeStartCounts = 0;
  int32_t forwardPreBrakeRightLimitCounts = 0;
  int32_t forwardPreBrakeLeftLimitCounts = 0;
  BoundedForwardBrakeTrigger forwardPreBrakeTrigger = BoundedForwardBrakeTrigger::NONE;
  bool forwardPreBrakeRightAtBrakeThreshold = false;
  bool forwardPreBrakeLeftAtBrakeThreshold = false;
  bool forwardPreBrakeRightAtHardLimit = false;
  bool forwardPreBrakeLeftAtHardLimit = false;
  bool forwardBrakeCommandCaptured = false;
  uint32_t forwardBrakeCommandTimestampMs = 0;
  MotionState forwardBrakeCommandState = MotionState::READY;
  int32_t forwardBrakeCommandRightCount = 0;
  int32_t forwardBrakeCommandLeftCount = 0;
  int forwardBrakeCommandRightPwm = 0;
  int forwardBrakeCommandLeftPwm = 0;
  bool forwardFirstPostBrakeLoopCaptured = false;
  uint32_t forwardFirstPostBrakeLoopTimestampMs = 0;
  MotionState forwardFirstPostBrakeLoopState = MotionState::READY;
  int32_t forwardFirstPostBrakeLoopRightCount = 0;
  int32_t forwardFirstPostBrakeLoopLeftCount = 0;
  int forwardFirstPostBrakeLoopRightPwm = 0;
  int forwardFirstPostBrakeLoopLeftPwm = 0;
  bool forwardFirstHardLimitGuardCaptured = false;
  uint32_t forwardFirstHardLimitGuardTimestampMs = 0;
  int32_t forwardFirstHardLimitGuardRightCount = 0;
  int32_t forwardFirstHardLimitGuardLeftCount = 0;
  bool forwardFirstHardLimitGuardRightExceeded = false;
  bool forwardFirstHardLimitGuardLeftExceeded = false;
  bool forwardFirstHardLimitGuardAlreadyBraking = false;
  CommandState state = CommandState::ACCEPTED;
  uint32_t startedAtMs = 0;
  uint32_t completedAtMs = 0;
};

MotionState motionState = MotionState::READY;
MotionDirection requestedDirection = MotionDirection::FORWARD;
MotionDirection activeDirection = MotionDirection::FORWARD;

float targetDistanceM = FORWARD_TARGET_DISTANCE_M;
uint32_t motionTimeoutMs = FORWARD_TIMEOUT_MS;
float targetTurnAngleRad = TURN_TARGET_ANGLE_RAD;

bool startRequested = false;
bool stopRequested = false;
bool squareRequested = false;
bool activeTurnUsesEffectiveTrack = false;

float requestedTargetDistanceM = FORWARD_TARGET_DISTANCE_M;
uint32_t requestedTimeoutMs = FORWARD_TIMEOUT_MS;
bool requestedTurnUsesEffectiveTrack = false;
float requestedTurnAngleRad = 0.0f;
int8_t activeCommandIndex = -1;
int8_t lastCommandIndex = -1;
CommandRecord commandHistory[COMMAND_HISTORY_CAPACITY];

SquareState squareState = SquareState::IDLE;
bool squareSegmentActive = false;
uint8_t squareSegmentsCompleted = 0;
uint32_t squareStartTime = 0;
uint32_t squareElapsedMs = 0;
uint32_t squarePauseStartTime = 0;
float squarePoseXM = 0.0f;
float squarePoseYM = 0.0f;
float squarePoseHeadingRad = 0.0f;
String squareFaultReason;

String faultReason;
String bootSessionId;
bool boundedFaultLatched = false;
String boundedFaultReason;

bool activeBoundedMotion = false;
BoundedMotionKind activeBoundedKind = BoundedMotionKind::NONE;
BoundedMotionProfile activeBoundedMotionProfile =
    BoundedMotionProfile::NONE;
int32_t activeBoundedTurnBrakeReserveCounts = 0;
bool activeBoundedRightIndividualBrakeStarted = false;
bool activeBoundedLeftIndividualBrakeStarted = false;
int32_t activeBoundedRightCountAtIndividualBrake = 0;
int32_t activeBoundedLeftCountAtIndividualBrake = 0;
int activeBoundedRightLastAppliedPwm = 0;
int activeBoundedLeftLastAppliedPwm = 0;
int32_t activeBoundedSyncErrorCounts = 0;
float activeBoundedRequestedTargetM = 0.0f;
uint32_t activeBoundedTimeoutMs = 0;
int32_t activeBoundedRightTargetCounts = 0;
int32_t activeBoundedLeftTargetCounts = 0;
int32_t activeBoundedRightLimitCounts = 0;
int32_t activeBoundedLeftLimitCounts = 0;
int32_t activeBoundedRightBrakeStartCounts = 0;
int32_t activeBoundedLeftBrakeStartCounts = 0;
int32_t activeBoundedRightCountAtBrakeStart = 0;
int32_t activeBoundedLeftCountAtBrakeStart = 0;
int32_t activeBoundedRightFinalCount = 0;
int32_t activeBoundedLeftFinalCount = 0;
bool activeBoundedBrakeStarted = false;
bool activeBoundedFaultPending = false;
BoundedStopMode activeBoundedStopMode = BoundedStopMode::NONE;
uint32_t activeBoundedBrakeStartTime = 0;
bool activeBoundedTargetReached = false;
bool activeBoundedInitialPulseCompleted = false;
uint8_t activeBoundedCorrectionPulsesUsed = 0;
uint8_t activeBoundedMaxCorrectionPulses = 0;
int32_t activeBoundedCountsBeforeCorrectionRight = 0;
int32_t activeBoundedCountsBeforeCorrectionLeft = 0;
bool activeBoundedCorrectionInProgress = false;
String activeBoundedBootSessionId;
float activeBoundedTurnMinSuccessAngleRad = 0.0f;
float activeBoundedTurnFinalAngleRad = 0.0f;
int32_t activeBoundedTurnMinWheelProgressCounts = 0;
uint32_t activeBoundedCorrectionPulseStartTime = 0;
int32_t activeBoundedStableRightCount = 0;
int32_t activeBoundedStableLeftCount = 0;
uint8_t activeBoundedStableSamples = 0;

uint32_t motionStartTime = 0;
uint32_t motionElapsedMs = 0;
uint32_t phaseStartTime = 0;
uint32_t previousControlTime = 0;
uint32_t coastStartTime = 0;

int32_t previousRightCount = 0;
int32_t previousLeftCount = 0;

int32_t rightCountAtStop = 0;
int32_t leftCountAtStop = 0;

bool rightBreakawayDetected = false;
bool leftBreakawayDetected = false;

int rightBreakawayPwm = 0;
int leftBreakawayPwm = 0;

float profileSpeedMps = 0.0f;
float rightIntegral = 0.0f;
float leftIntegral = 0.0f;

float rightSpeedMps = 0.0f;
float leftSpeedMps = 0.0f;

float rightDistanceM = 0.0f;
float leftDistanceM = 0.0f;
float averageDistanceM = 0.0f;
float headingRad = 0.0f;
float stopMarginAtStopM = 0.0f;

int rightPwm = 0;
int leftPwm = 0;

int rightPwmAtStop = 0;
int leftPwmAtStop = 0;

uint8_t rightStallCycles = 0;
uint8_t leftStallCycles = 0;

int32_t lastRightDelta = 0;
int32_t lastLeftDelta = 0;

// ---------------------------------------------------------------------------
// Encoder ISRs
// ---------------------------------------------------------------------------

void IRAM_ATTR onRightEncoderA()
{
  rightRawCount += digitalRead(RIGHT_B) ? 1 : -1;
}

void IRAM_ATTR onLeftEncoderA()
{
  leftRawCount += digitalRead(LEFT_B) ? 1 : -1;
}

// ---------------------------------------------------------------------------
// Motor and encoder helpers
// ---------------------------------------------------------------------------

void stopMotors()
{
  analogWrite(ZK_D0, 0);
  analogWrite(ZK_D1, 0);
  analogWrite(ZK_D2, 0);
  analogWrite(ZK_D3, 0);

  rightPwm = 0;
  leftPwm = 0;
}

void brakeMotors()
{
  // TA6586 brake mode: both inputs HIGH, both bridge outputs LOW.
  analogWrite(ZK_D0, 255);
  analogWrite(ZK_D1, 255);
  analogWrite(ZK_D2, 255);
  analogWrite(ZK_D3, 255);

  rightPwm = 0;
  leftPwm = 0;
}

void setDrivePwm(int requestedRightPwm, int requestedLeftPwm)
{
  requestedRightPwm = constrain(requestedRightPwm, 0, 255);
  requestedLeftPwm = constrain(requestedLeftPwm, 0, 255);

  const bool rightForward =
      activeDirection == MotionDirection::FORWARD ||
      activeDirection == MotionDirection::TURN_LEFT;

  const bool leftForward =
      activeDirection == MotionDirection::FORWARD ||
      activeDirection == MotionDirection::TURN_RIGHT;

  if (rightForward) {
    analogWrite(ZK_D0, 0);
    analogWrite(ZK_D1, requestedRightPwm);
  } else {
    analogWrite(ZK_D1, 0);
    analogWrite(ZK_D0, requestedRightPwm);
  }

  if (leftForward) {
    analogWrite(ZK_D2, 0);
    analogWrite(ZK_D3, requestedLeftPwm);
  } else {
    analogWrite(ZK_D3, 0);
    analogWrite(ZK_D2, requestedLeftPwm);
  }

  rightPwm = requestedRightPwm;
  leftPwm = requestedLeftPwm;
}

CommandRecord* activeCommand();

void applyMicroTurnOutputs(
    int requestedRightPwm,
    int requestedLeftPwm,
    bool rightBrake,
    bool leftBrake)
{
  const int rightOutput = rightBrake ? 0 : constrain(requestedRightPwm, 0, 255);
  const int leftOutput = leftBrake ? 0 : constrain(requestedLeftPwm, 0, 255);
  const bool rightForward =
      activeDirection == MotionDirection::FORWARD ||
      activeDirection == MotionDirection::TURN_LEFT;
  const bool leftForward =
      activeDirection == MotionDirection::FORWARD ||
      activeDirection == MotionDirection::TURN_RIGHT;

  if (rightBrake) {
    // Confirmed TA6586 active brake for this motor pair: both inputs HIGH.
    analogWrite(ZK_D0, 255);
    analogWrite(ZK_D1, 255);
  } else if (rightForward) {
    analogWrite(ZK_D0, 0);
    analogWrite(ZK_D1, rightOutput);
  } else {
    analogWrite(ZK_D1, 0);
    analogWrite(ZK_D0, rightOutput);
  }

  if (leftBrake) {
    // Same confirmed active-brake combination for the left motor pair.
    analogWrite(ZK_D2, 255);
    analogWrite(ZK_D3, 255);
  } else if (leftForward) {
    analogWrite(ZK_D2, 0);
    analogWrite(ZK_D3, leftOutput);
  } else {
    analogWrite(ZK_D3, 0);
    analogWrite(ZK_D2, leftOutput);
  }

  rightPwm = rightOutput;
  leftPwm = leftOutput;
  if (!rightBrake && rightOutput > 0) {
    activeBoundedRightLastAppliedPwm = rightOutput;
  }
  if (!leftBrake && leftOutput > 0) {
    activeBoundedLeftLastAppliedPwm = leftOutput;
  }
  CommandRecord* command = activeCommand();
  if (command != nullptr) {
    if (!rightBrake && rightOutput > 0) {
      command->rightLastAppliedPwm = rightOutput;
    }
    if (!leftBrake && leftOutput > 0) {
      command->leftLastAppliedPwm = leftOutput;
    }
  }
}

void resetEncoderCounts()
{
  noInterrupts();
  rightRawCount = 0;
  leftRawCount = 0;
  interrupts();
}

void readMotionCounts(int32_t &rightMotion, int32_t &leftMotion)
{
  noInterrupts();
  const int32_t rightRaw = rightRawCount;
  const int32_t leftRaw = leftRawCount;
  interrupts();

  // Powered tests established these signs for physical forward motion.
  const int32_t rightForward = -rightRaw;
  const int32_t leftForward = leftRaw;

  const int32_t rightSign =
      (activeDirection == MotionDirection::FORWARD ||
       activeDirection == MotionDirection::TURN_LEFT) ? 1 : -1;

  const int32_t leftSign =
      (activeDirection == MotionDirection::FORWARD ||
       activeDirection == MotionDirection::TURN_RIGHT) ? 1 : -1;

  rightMotion = rightForward * rightSign;
  leftMotion = leftForward * leftSign;
}

int32_t rightPhysicalSign()
{
  return (activeDirection == MotionDirection::FORWARD ||
          activeDirection == MotionDirection::TURN_LEFT) ? 1 : -1;
}

int32_t leftPhysicalSign()
{
  return (activeDirection == MotionDirection::FORWARD ||
          activeDirection == MotionDirection::TURN_RIGHT) ? 1 : -1;
}

float activeTrackWidthM()
{
  const bool turning =
      activeDirection == MotionDirection::TURN_LEFT ||
      activeDirection == MotionDirection::TURN_RIGHT;

  return turning && activeTurnUsesEffectiveTrack
             ? TURN_EFFECTIVE_TRACK_WIDTH_M
             : TRACK_WIDTH_M;
}

void updateDistancesFromEncoders()
{
  int32_t rightCount;
  int32_t leftCount;
  readMotionCounts(rightCount, leftCount);

  rightDistanceM = rightCount * RIGHT_M_PER_COUNT;
  leftDistanceM = leftCount * LEFT_M_PER_COUNT;

  averageDistanceM =
      (rightDistanceM + leftDistanceM) * 0.5f;

  const float rightPhysicalDistance =
      rightDistanceM * rightPhysicalSign();

  const float leftPhysicalDistance =
      leftDistanceM * leftPhysicalSign();

  headingRad =
      (rightPhysicalDistance - leftPhysicalDistance) / activeTrackWidthM();
}

const char* stateName()
{
  switch (motionState) {
    case MotionState::READY:    return "READY";
    case MotionState::STARTING: return "STARTING";
    case MotionState::DRIVING:  return "DRIVING";
    case MotionState::BRAKING:  return "BRAKING";
    case MotionState::CORRECTING: return "CORRECTING";
    case MotionState::COASTING: return "COASTING";
    case MotionState::SUCCESS:  return "SUCCESS";
    case MotionState::FAULT:    return "FAULT";
  }

  return "UNKNOWN";
}

const char* commandStateName(CommandState state)
{
  switch (state) {
    case CommandState::ACCEPTED: return "ACCEPTED";
    case CommandState::STARTING: return "STARTING";
    case CommandState::DRIVING:  return "DRIVING";
    case CommandState::BRAKING:  return "BRAKING";
    case CommandState::COASTING: return "COASTING";
    case CommandState::SUCCESS:  return "SUCCESS";
    case CommandState::PARTIAL_PROGRESS: return "PARTIAL_PROGRESS";
    case CommandState::FAULT:    return "FAULT";
    case CommandState::STOPPED:  return "STOPPED";
  }
  return "FAULT";
}

const char* boundedStopModeName(BoundedStopMode mode)
{
  switch (mode) {
    case BoundedStopMode::NONE:         return "NONE";
    case BoundedStopMode::ACTIVE_BRAKE: return "ACTIVE_BRAKE";
    case BoundedStopMode::COAST:        return "COAST";
  }
  return "NONE";
}

const char* motionStateName(MotionState state)
{
  switch (state) {
    case MotionState::READY:    return "READY";
    case MotionState::STARTING: return "STARTING";
    case MotionState::DRIVING:  return "DRIVING";
    case MotionState::BRAKING:  return "BRAKING";
    case MotionState::CORRECTING: return "CORRECTING";
    case MotionState::COASTING: return "COASTING";
    case MotionState::SUCCESS:  return "SUCCESS";
    case MotionState::FAULT:    return "FAULT";
  }
  return "UNKNOWN";
}

const char* boundedForwardBrakeTriggerName(BoundedForwardBrakeTrigger trigger)
{
  switch (trigger) {
    case BoundedForwardBrakeTrigger::NONE:             return "NONE";
    case BoundedForwardBrakeTrigger::PREDICTIVE_BRAKE: return "PREDICTIVE_BRAKE";
    case BoundedForwardBrakeTrigger::TARGET_REACHED:   return "TARGET_REACHED";
    case BoundedForwardBrakeTrigger::HARD_LIMIT:       return "HARD_LIMIT";
  }
  return "NONE";
}

const char* boundedMotionProfileName(BoundedMotionProfile profile)
{
  switch (profile) {
    case BoundedMotionProfile::NONE:                   return "NONE";
    case BoundedMotionProfile::BOUNDED_FORWARD_V1:     return "BOUNDED_FORWARD_V1";
    case BoundedMotionProfile::BOUNDED_MICRO_TURN_V1:  return "BOUNDED_MICRO_TURN_V1";
  }
  return "NONE";
}

CommandRecord* activeCommand()
{
  return activeCommandIndex < 0 ? nullptr : &commandHistory[activeCommandIndex];
}

CommandRecord* lastCommand()
{
  return lastCommandIndex < 0 ? nullptr : &commandHistory[lastCommandIndex];
}

bool activeBoundedForwardCommand(const CommandRecord* command)
{
  return command != nullptr &&
         command->boundedMotionProfile ==
             BoundedMotionProfile::BOUNDED_FORWARD_V1;
}

void captureBoundedForwardPreBrakeGuard(
    int32_t rightCount,
    int32_t leftCount,
    BoundedForwardBrakeTrigger trigger,
    bool rightAtBrakeThreshold,
    bool leftAtBrakeThreshold,
    bool rightAtHardLimit,
    bool leftAtHardLimit)
{
  CommandRecord* command = activeCommand();
  if (!activeBoundedForwardCommand(command) ||
      command->forwardPreBrakeGuardCaptured) {
    return;
  }
  command->forwardPreBrakeGuardCaptured = true;
  command->forwardPreBrakeGuardTimestampMs = millis();
  command->forwardPreBrakeGuardState = motionState;
  command->forwardPreBrakeRightCount = rightCount;
  command->forwardPreBrakeLeftCount = leftCount;
  command->forwardPreBrakeRightBrakeStartCounts =
      activeBoundedRightBrakeStartCounts;
  command->forwardPreBrakeLeftBrakeStartCounts =
      activeBoundedLeftBrakeStartCounts;
  command->forwardPreBrakeRightLimitCounts = activeBoundedRightLimitCounts;
  command->forwardPreBrakeLeftLimitCounts = activeBoundedLeftLimitCounts;
  command->forwardPreBrakeTrigger = trigger;
  command->forwardPreBrakeRightAtBrakeThreshold = rightAtBrakeThreshold;
  command->forwardPreBrakeLeftAtBrakeThreshold = leftAtBrakeThreshold;
  command->forwardPreBrakeRightAtHardLimit = rightAtHardLimit;
  command->forwardPreBrakeLeftAtHardLimit = leftAtHardLimit;
}

void captureBoundedForwardBrakeCommand(
    int32_t rightCount,
    int32_t leftCount)
{
  CommandRecord* command = activeCommand();
  if (!activeBoundedForwardCommand(command) ||
      command->forwardBrakeCommandCaptured) {
    return;
  }
  command->forwardBrakeCommandCaptured = true;
  command->forwardBrakeCommandTimestampMs = millis();
  command->forwardBrakeCommandState = motionState;
  command->forwardBrakeCommandRightCount = rightCount;
  command->forwardBrakeCommandLeftCount = leftCount;
  command->forwardBrakeCommandRightPwm = rightPwm;
  command->forwardBrakeCommandLeftPwm = leftPwm;
}

void captureBoundedForwardFirstPostBrakeLoop(
    int32_t rightCount,
    int32_t leftCount)
{
  CommandRecord* command = activeCommand();
  if (!activeBoundedForwardCommand(command) ||
      !activeBoundedBrakeStarted ||
      command->forwardFirstPostBrakeLoopCaptured) {
    return;
  }
  command->forwardFirstPostBrakeLoopCaptured = true;
  command->forwardFirstPostBrakeLoopTimestampMs = millis();
  command->forwardFirstPostBrakeLoopState = motionState;
  command->forwardFirstPostBrakeLoopRightCount = rightCount;
  command->forwardFirstPostBrakeLoopLeftCount = leftCount;
  command->forwardFirstPostBrakeLoopRightPwm = rightPwm;
  command->forwardFirstPostBrakeLoopLeftPwm = leftPwm;
}

void captureBoundedForwardFirstHardLimitGuard(
    int32_t rightCount,
    int32_t leftCount,
    bool rightExceeded,
    bool leftExceeded)
{
  CommandRecord* command = activeCommand();
  if (!activeBoundedForwardCommand(command) ||
      command->forwardFirstHardLimitGuardCaptured) {
    return;
  }
  command->forwardFirstHardLimitGuardCaptured = true;
  command->forwardFirstHardLimitGuardTimestampMs = millis();
  command->forwardFirstHardLimitGuardRightCount = rightCount;
  command->forwardFirstHardLimitGuardLeftCount = leftCount;
  command->forwardFirstHardLimitGuardRightExceeded = rightExceeded;
  command->forwardFirstHardLimitGuardLeftExceeded = leftExceeded;
  command->forwardFirstHardLimitGuardAlreadyBraking =
      motionState == MotionState::BRAKING;
}

void setActiveCommandState(CommandState state)
{
  CommandRecord* command = activeCommand();
  if (command != nullptr) {
    command->state = state;
  }
}

void completeActiveCommand(CommandState state)
{
  if (activeBoundedMotion && state == CommandState::FAULT) {
    boundedFaultLatched = true;
    boundedFaultReason = faultReason;
  }

  CommandRecord* command = activeCommand();
  if (command != nullptr) {
    command->state = state;
    command->completedAtMs = millis();
    lastCommandIndex = activeCommandIndex;
    activeCommandIndex = -1;
  }

  activeBoundedMotion = false;
  activeBoundedKind = BoundedMotionKind::NONE;
  activeBoundedMotionProfile = BoundedMotionProfile::NONE;
  activeBoundedTurnBrakeReserveCounts = 0;
  activeBoundedRightIndividualBrakeStarted = false;
  activeBoundedLeftIndividualBrakeStarted = false;
  activeBoundedRightCountAtIndividualBrake = 0;
  activeBoundedLeftCountAtIndividualBrake = 0;
  activeBoundedRightLastAppliedPwm = 0;
  activeBoundedLeftLastAppliedPwm = 0;
  activeBoundedSyncErrorCounts = 0;
  activeBoundedRequestedTargetM = 0.0f;
  activeBoundedTimeoutMs = 0;
  activeBoundedRightTargetCounts = 0;
  activeBoundedLeftTargetCounts = 0;
  activeBoundedRightLimitCounts = 0;
  activeBoundedLeftLimitCounts = 0;
  activeBoundedRightBrakeStartCounts = 0;
  activeBoundedLeftBrakeStartCounts = 0;
  activeBoundedRightCountAtBrakeStart = 0;
  activeBoundedLeftCountAtBrakeStart = 0;
  activeBoundedRightFinalCount = 0;
  activeBoundedLeftFinalCount = 0;
  activeBoundedBrakeStarted = false;
  activeBoundedFaultPending = false;
  activeBoundedStopMode = BoundedStopMode::NONE;
  activeBoundedBrakeStartTime = 0;
  activeBoundedTargetReached = false;
  activeBoundedInitialPulseCompleted = false;
  activeBoundedCorrectionPulsesUsed = 0;
  activeBoundedMaxCorrectionPulses = 0;
  activeBoundedCountsBeforeCorrectionRight = 0;
  activeBoundedCountsBeforeCorrectionLeft = 0;
  activeBoundedCorrectionInProgress = false;
  activeBoundedBootSessionId = "";
  activeBoundedCorrectionPulseStartTime = 0;
  activeBoundedStableSamples = 0;
  activeBoundedTurnMinSuccessAngleRad = 0.0f;
  activeBoundedTurnFinalAngleRad = 0.0f;
  activeBoundedTurnMinWheelProgressCounts = 0;
}

bool commandIdIsValid(const String &commandId)
{
  if (commandId.length() < 1 || commandId.length() > 64) {
    return false;
  }

  for (size_t index = 0; index < commandId.length(); ++index) {
    const char value = commandId[index];
    const bool allowed =
        (value >= 'A' && value <= 'Z') ||
        (value >= 'a' && value <= 'z') ||
        (value >= '0' && value <= '9') ||
        value == '.' || value == '_' || value == '-';
    if (!allowed) {
      return false;
    }
  }
  return true;
}

String generateBootSessionId()
{
  const uint32_t high = esp_random();
  const uint32_t low = esp_random();
  char value[17];
  snprintf(
      value,
      sizeof(value),
      "%08lX%08lX",
      static_cast<unsigned long>(high),
      static_cast<unsigned long>(low)
  );
  return String(value);
}

bool parseFiniteFloatStrict(const String &text, float &value)
{
  if (text.length() == 0) {
    return false;
  }

  for (size_t index = 0; index < text.length(); ++index) {
    if (isspace(static_cast<unsigned char>(text[index]))) {
      return false;
    }
  }

  char *end = nullptr;
  const float parsed = strtof(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0' || !isfinite(parsed)) {
    return false;
  }

  value = parsed;
  return true;
}

int8_t findCommand(const String &commandId)
{
  for (uint8_t index = 0; index < COMMAND_HISTORY_CAPACITY; ++index) {
    if (commandHistory[index].used &&
        commandHistory[index].commandId == commandId) {
      return static_cast<int8_t>(index);
    }
  }
  return -1;
}

int8_t reserveCommandRecord()
{
  for (uint8_t index = 0; index < COMMAND_HISTORY_CAPACITY; ++index) {
    if (!commandHistory[index].used) {
      commandHistory[index] = CommandRecord();
      commandHistory[index].used = true;
      return static_cast<int8_t>(index);
    }
  }
  return -1;
}

const char* directionName()
{
  switch (activeDirection) {
    case MotionDirection::FORWARD:    return "FORWARD";
    case MotionDirection::REVERSE:    return "REVERSE";
    case MotionDirection::TURN_LEFT:  return "TURN_LEFT";
    case MotionDirection::TURN_RIGHT: return "TURN_RIGHT";
  }

  return "UNKNOWN";
}

const char* squareStateName()
{
  switch (squareState) {
    case SquareState::IDLE:    return "IDLE";
    case SquareState::RUNNING: return "RUNNING";
    case SquareState::SUCCESS: return "SUCCESS";
    case SquareState::FAULT:   return "FAULT";
  }

  return "UNKNOWN";
}

bool squareIsActive()
{
  return squareState == SquareState::RUNNING;
}

const char* squareCurrentActionName()
{
  if (squareState == SquareState::SUCCESS) {
    return "DONE";
  }

  if (squareState == SquareState::FAULT) {
    return "STOPPED";
  }

  if (!squareIsActive()) {
    return "NONE";
  }

  if (!squareSegmentActive) {
    return "SETTLING";
  }

  return directionName();
}

float normalizeAngleRad(float angle)
{
  while (angle > PI) {
    angle -= 2.0f * PI;
  }

  while (angle < -PI) {
    angle += 2.0f * PI;
  }

  return angle;
}

bool isTurnCommand()
{
  return activeDirection == MotionDirection::TURN_LEFT ||
         activeDirection == MotionDirection::TURN_RIGHT;
}

float activeMaxProfileSpeedMps()
{
  return isTurnCommand()
             ? TURN_MAX_PROFILE_SPEED_M_S
             : MAX_PROFILE_SPEED_M_S;
}

int rightSustainPwm()
{
  return isTurnCommand()
             ? TURN_SUSTAIN_PWM
             : RIGHT_RUN_PWM_AT_MAX;
}

int leftSustainPwm()
{
  return isTurnCommand()
             ? TURN_SUSTAIN_PWM
             : LEFT_RUN_PWM_AT_MAX;
}

bool motionIsActive()
{
  return motionState == MotionState::STARTING ||
         motionState == MotionState::DRIVING ||
         motionState == MotionState::CORRECTING ||
         motionState == MotionState::BRAKING ||
         motionState == MotionState::COASTING;
}

float currentStopMarginM()
{
  const float averageSpeedMps =
      (fabsf(rightSpeedMps) + fabsf(leftSpeedMps)) * 0.5f;

  return constrain(
      BRAKE_DISTANCE_GAIN_S2_PER_M *
          averageSpeedMps * averageSpeedMps,
      isTurnCommand() ? TURN_MIN_STOP_MARGIN_M : MIN_STOP_MARGIN_M,
      isTurnCommand() ? TURN_MAX_STOP_MARGIN_M : MAX_STOP_MARGIN_M
  );
}

int32_t encoderCountsForTravelM(float travelM, float metersPerCount)
{
  return max(
      static_cast<int32_t>(1),
      static_cast<int32_t>(ceilf(travelM / metersPerCount))
  );
}

int32_t boundedBrakeStartCounts(
    int32_t targetCounts,
    float metersPerCount,
    float stopMarginM)
{
  const int32_t reserveCounts = encoderCountsForTravelM(
      stopMarginM,
      metersPerCount
  );
  return max(static_cast<int32_t>(1), targetCounts - reserveCounts);
}

int32_t boundedMicroTurnBrakeReserveCounts(
    float metersPerCount,
    float stopMarginM)
{
  const int32_t dynamicReserveCounts = encoderCountsForTravelM(
      stopMarginM,
      metersPerCount
  );
  return max(
      BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS,
      dynamicReserveCounts
  );
}

int32_t boundedMicroTurnBrakeStartCounts(
    int32_t targetCounts,
    float metersPerCount,
    float stopMarginM)
{
  const int32_t reserveCounts = boundedMicroTurnBrakeReserveCounts(
      metersPerCount,
      stopMarginM
  );
  return max(static_cast<int32_t>(1), targetCounts - reserveCounts);
}

int32_t boundedCompletionToleranceCounts(int32_t targetCounts)
{
  const int32_t proportionalCounts = static_cast<int32_t>(ceilf(
      static_cast<float>(targetCounts) * 0.20f
  ));
  return min(
      static_cast<int32_t>(6),
      max(static_cast<int32_t>(1), proportionalCounts)
  );
}

int32_t boundedMicroTurnCompletionToleranceCounts(int32_t targetCounts)
{
  const int32_t proportionalCounts = static_cast<int32_t>(ceilf(
      static_cast<float>(targetCounts) * 0.30f
  ));
  return min(
      static_cast<int32_t>(4),
      max(static_cast<int32_t>(1), proportionalCounts)
  );
}

int boundedBreakawayPwmCeiling()
{
  if (!activeBoundedMotion) {
    return BREAKAWAY_MAX_PWM;
  }

  const int32_t smallestBrakeStart = min(
      activeBoundedRightBrakeStartCounts,
      activeBoundedLeftBrakeStartCounts
  );
  return constrain(
      BREAKAWAY_START_PWM +
          static_cast<int>(smallestBrakeStart) * BREAKAWAY_PWM_STEP,
      BREAKAWAY_START_PWM,
      BREAKAWAY_MAX_PWM
  );
}

int32_t boundedBreakawayThresholdCounts()
{
  if (!activeBoundedMotion) {
    return BREAKAWAY_DETECT_COUNTS;
  }

  const int32_t smallestTarget = min(
      activeBoundedRightTargetCounts,
      activeBoundedLeftTargetCounts
  );
  const int32_t smallestBrakeStart = min(
      activeBoundedRightBrakeStartCounts,
      activeBoundedLeftBrakeStartCounts
  );
  // A short bounded request must leave enough encoder room for predictive
  // braking. Do not wait for the legacy 15-count breakaway threshold.
  return max(
      static_cast<int32_t>(1),
      min(smallestTarget / 2, smallestBrakeStart / 2)
  );
}

void configureBoundedCommand(
    CommandRecord &command,
    MotionDirection direction,
    float requestedTargetM,
    uint32_t timeoutMs)
{
  command.boundedKind = direction == MotionDirection::FORWARD
      ? BoundedMotionKind::LINEAR
      : BoundedMotionKind::TURN;
  command.rightTargetCounts =
      encoderCountsForTravelM(requestedTargetM, RIGHT_M_PER_COUNT);
  command.leftTargetCounts =
      encoderCountsForTravelM(requestedTargetM, LEFT_M_PER_COUNT);
  // One count is a quantization allowance, not a claim of zero mechanical
  // overshoot. The hard encoder cutoff still occurs before more drive PWM.
  command.rightLimitCounts = command.rightTargetCounts + 1;
  command.leftLimitCounts = command.leftTargetCounts + 1;
  const bool turn = direction == MotionDirection::TURN_LEFT ||
                    direction == MotionDirection::TURN_RIGHT;
  command.rightCompletionToleranceCounts = turn
      ? boundedMicroTurnCompletionToleranceCounts(command.rightTargetCounts)
      : boundedCompletionToleranceCounts(command.rightTargetCounts);
  command.leftCompletionToleranceCounts = turn
      ? boundedMicroTurnCompletionToleranceCounts(command.leftTargetCounts)
      : boundedCompletionToleranceCounts(command.leftTargetCounts);
  command.rightMinSuccessCounts = command.rightTargetCounts -
      command.rightCompletionToleranceCounts;
  command.leftMinSuccessCounts = command.leftTargetCounts -
      command.leftCompletionToleranceCounts;
  command.boundedMotionProfile = turn
      ? BoundedMotionProfile::BOUNDED_MICRO_TURN_V1
      : BoundedMotionProfile::BOUNDED_FORWARD_V1;
  command.turnBrakeReserveCounts = turn
      ? BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS
      : 0;
  command.rightIndividualBrakeStarted = false;
  command.leftIndividualBrakeStarted = false;
  command.rightCountAtIndividualBrake = 0;
  command.leftCountAtIndividualBrake = 0;
  command.rightLastAppliedPwm = 0;
  command.leftLastAppliedPwm = 0;
  command.syncErrorCounts = 0;
  command.boundedActualProgressRatio = 0.0f;
  command.reobserveRequired = false;
  const float initialStopMarginM = turn
      ? TURN_MIN_STOP_MARGIN_M
      : MIN_STOP_MARGIN_M;
  if (turn) {
    command.rightBrakeStartCounts = boundedMicroTurnBrakeStartCounts(
        command.rightTargetCounts,
        RIGHT_M_PER_COUNT,
        initialStopMarginM
    );
    command.leftBrakeStartCounts = boundedMicroTurnBrakeStartCounts(
        command.leftTargetCounts,
        LEFT_M_PER_COUNT,
        initialStopMarginM
    );
  } else {
    command.rightBrakeStartCounts = boundedBrakeStartCounts(
        command.rightTargetCounts,
        RIGHT_M_PER_COUNT,
        initialStopMarginM
    );
    command.leftBrakeStartCounts = boundedBrakeStartCounts(
        command.leftTargetCounts,
        LEFT_M_PER_COUNT,
        initialStopMarginM
    );
  }
  command.boundedTimeoutMs = timeoutMs;
  command.boundedInitialPulseCompleted = false;
  command.boundedCorrectionPulsesUsed = 0;
  command.boundedMaxCorrectionPulses =
      command.boundedMotionProfile == BoundedMotionProfile::BOUNDED_MICRO_TURN_V1
          ? BOUNDED_MICRO_TURN_MAX_CORRECTION_PULSES
          : 0;
  command.boundedCountsBeforeCorrectionRight = 0;
  command.boundedCountsBeforeCorrectionLeft = 0;

  activeBoundedMotion = true;
  activeBoundedKind = command.boundedKind;
  activeBoundedMotionProfile = command.boundedMotionProfile;
  activeBoundedTurnBrakeReserveCounts = command.turnBrakeReserveCounts;
  activeBoundedRightIndividualBrakeStarted = false;
  activeBoundedLeftIndividualBrakeStarted = false;
  activeBoundedRightCountAtIndividualBrake = 0;
  activeBoundedLeftCountAtIndividualBrake = 0;
  activeBoundedRightLastAppliedPwm = 0;
  activeBoundedLeftLastAppliedPwm = 0;
  activeBoundedSyncErrorCounts = 0;
  activeBoundedRequestedTargetM = requestedTargetM;
  activeBoundedTimeoutMs = timeoutMs;
  activeBoundedRightTargetCounts = command.rightTargetCounts;
  activeBoundedLeftTargetCounts = command.leftTargetCounts;
  activeBoundedRightLimitCounts = command.rightLimitCounts;
  activeBoundedLeftLimitCounts = command.leftLimitCounts;
  activeBoundedRightBrakeStartCounts = command.rightBrakeStartCounts;
  activeBoundedLeftBrakeStartCounts = command.leftBrakeStartCounts;
  activeBoundedRightCountAtBrakeStart = 0;
  activeBoundedLeftCountAtBrakeStart = 0;
  activeBoundedRightFinalCount = 0;
  activeBoundedLeftFinalCount = 0;
  activeBoundedBrakeStarted = false;
  activeBoundedFaultPending = false;
  activeBoundedStopMode = BoundedStopMode::NONE;
  activeBoundedBrakeStartTime = 0;
  activeBoundedTargetReached = false;
  activeBoundedInitialPulseCompleted = false;
  activeBoundedCorrectionPulsesUsed = 0;
  activeBoundedMaxCorrectionPulses = 0;
  activeBoundedCountsBeforeCorrectionRight = 0;
  activeBoundedCountsBeforeCorrectionLeft = 0;
  activeBoundedCorrectionInProgress = false;
  activeBoundedBootSessionId = bootSessionId;
  activeBoundedCorrectionPulseStartTime = 0;
  activeBoundedStableSamples = 0;

  activeBoundedMaxCorrectionPulses = command.boundedMaxCorrectionPulses;
  activeBoundedTurnMinSuccessAngleRad = turn
      ? fabsf(requestedTargetM / (TURN_EFFECTIVE_TRACK_WIDTH_M * 0.5f)) * 0.70f
      : 0.0f;
  activeBoundedTurnFinalAngleRad = 0.0f;
  activeBoundedTurnMinWheelProgressCounts = turn
      ? max(static_cast<int32_t>(1), static_cast<int32_t>(ceilf(
            static_cast<float>(min(
                command.rightTargetCounts,
                command.leftTargetCounts
            )) * 0.50f
        )))
      : 0;
  command.boundedTurnMinSuccessAngleRad = activeBoundedTurnMinSuccessAngleRad;
  command.boundedTurnMinWheelProgressCounts =
      activeBoundedTurnMinWheelProgressCounts;
  command.boundedInitialPulseCompleted = false;
  command.boundedCorrectionPulsesUsed = 0;
  command.boundedMaxCorrectionPulses = activeBoundedMaxCorrectionPulses;
  command.boundedCountsBeforeCorrectionRight = 0;
  command.boundedCountsBeforeCorrectionLeft = 0;
  command.boundedTurnFinalAngleRad = 0.0f;
  command.boundedActualProgressRatio = 0.0f;
  command.reobserveRequired = false;
}

// ---------------------------------------------------------------------------
// Motion-state transitions
// ---------------------------------------------------------------------------

void beginBoundedBraking(bool faultPending, const String &reason);

void abortMotion(const String &reason)
{
  if (activeBoundedMotion && motionIsActive()) {
    // Every bounded fault uses the verified electrical brake before it becomes
    // terminal. This path deliberately never re-enters directed drive output.
    if (!activeBoundedFaultPending) {
      beginBoundedBraking(true, reason);
    }
    return;
  }

  rightPwmAtStop = rightPwm;
  leftPwmAtStop = leftPwm;

  stopMotors();
  updateDistancesFromEncoders();
  readMotionCounts(rightCountAtStop, leftCountAtStop);

  motionElapsedMs = motionStartTime == 0 ? 0 : millis() - motionStartTime;

  rightSpeedMps = 0.0f;
  leftSpeedMps = 0.0f;
  profileSpeedMps = 0.0f;

  faultReason = reason;
  motionState = MotionState::FAULT;
  completeActiveCommand(CommandState::FAULT);

  Serial.print("MOTION_FAULT=");
  Serial.println(reason);
}

void startMotion(
    MotionDirection direction,
    float requestedTarget,
    uint32_t requestedTimeout,
    bool useEffectiveTurnTrack,
    float requestedTurnAngleRad)
{
  stopMotors();
  activeDirection = direction;
  activeTurnUsesEffectiveTrack = useEffectiveTurnTrack;
  targetDistanceM = requestedTarget;
  motionTimeoutMs = requestedTimeout;
  targetTurnAngleRad = requestedTurnAngleRad;

  resetEncoderCounts();

  faultReason = "";
  stopRequested = false;

  rightBreakawayDetected = false;
  leftBreakawayDetected = false;

  rightBreakawayPwm = 0;
  leftBreakawayPwm = 0;

  profileSpeedMps = 0.0f;
  rightIntegral = 0.0f;
  leftIntegral = 0.0f;

  rightSpeedMps = 0.0f;
  leftSpeedMps = 0.0f;

  rightDistanceM = 0.0f;
  leftDistanceM = 0.0f;
  averageDistanceM = 0.0f;
  headingRad = 0.0f;
  stopMarginAtStopM = 0.0f;

  previousRightCount = 0;
  previousLeftCount = 0;

  rightCountAtStop = 0;
  leftCountAtStop = 0;

  rightPwmAtStop = 0;
  leftPwmAtStop = 0;

  rightStallCycles = 0;
  leftStallCycles = 0;

  lastRightDelta = 0;
  lastLeftDelta = 0;

  motionStartTime = millis();
  motionElapsedMs = 0;
  phaseStartTime = motionStartTime;
  previousControlTime = motionStartTime;

  motionState = MotionState::STARTING;
  CommandRecord* command = activeCommand();
  if (command != nullptr) {
    command->startedAtMs = motionStartTime;
    setActiveCommandState(CommandState::STARTING);
  }

  Serial.print("MOTION_STARTING direction=");
  Serial.println(directionName());
}

void transitionToDriving(uint32_t now)
{
  int32_t rightCount;
  int32_t leftCount;
  readMotionCounts(rightCount, leftCount);

  previousRightCount = rightCount;
  previousLeftCount = leftCount;

  previousControlTime = now;
  phaseStartTime = now;

  // The adaptive launch has already accelerated the drivetrain. Begin close
  // to the loaded cruise region instead of restarting the ramp from zero.
  profileSpeedMps = isTurnCommand() ? 0.065f : 0.100f;

  rightIntegral = 0.0f;
  leftIntegral = 0.0f;

  rightStallCycles = 0;
  leftStallCycles = 0;

  setDrivePwm(rightSustainPwm(), leftSustainPwm());

  motionState = MotionState::DRIVING;
  setActiveCommandState(CommandState::DRIVING);

  Serial.println("MOTION_DRIVING");
}

void beginCoasting()
{
  rightPwmAtStop = rightPwm;
  leftPwmAtStop = leftPwm;
  stopMarginAtStopM = currentStopMarginM();

  brakeMotors();
  readMotionCounts(rightCountAtStop, leftCountAtStop);

  rightSpeedMps = 0.0f;
  leftSpeedMps = 0.0f;
  profileSpeedMps = 0.0f;

  coastStartTime = millis();
  motionState = MotionState::COASTING;
  setActiveCommandState(CommandState::COASTING);

  Serial.println("MOTION_BRAKING");
}

void beginBoundedTargetSettling()
{
  beginBoundedBraking(false, "");
}

void beginBoundedBraking(bool faultPending, const String &reason)
{
  if (!activeBoundedMotion) {
    return;
  }

  if (!activeBoundedBrakeStarted) {
    rightPwmAtStop = rightPwm;
    leftPwmAtStop = leftPwm;
    stopMarginAtStopM = currentStopMarginM();

    // Explicitly cut directed drive before engaging the known TA6586 brake
    // state. From here until completion no path may issue directed drive PWM.
    stopMotors();
    brakeMotors();
    readMotionCounts(
        activeBoundedRightCountAtBrakeStart,
        activeBoundedLeftCountAtBrakeStart
    );
    captureBoundedForwardBrakeCommand(
        activeBoundedRightCountAtBrakeStart,
        activeBoundedLeftCountAtBrakeStart
    );
    rightCountAtStop = activeBoundedRightCountAtBrakeStart;
    leftCountAtStop = activeBoundedLeftCountAtBrakeStart;

    CommandRecord* command = activeCommand();
    if (command != nullptr) {
      command->rightBrakeStartCounts = activeBoundedRightBrakeStartCounts;
      command->leftBrakeStartCounts = activeBoundedLeftBrakeStartCounts;
      command->rightCountAtBrakeStart = activeBoundedRightCountAtBrakeStart;
      command->leftCountAtBrakeStart = activeBoundedLeftCountAtBrakeStart;
      command->stopMode = BoundedStopMode::ACTIVE_BRAKE;
    }

    activeBoundedBrakeStarted = true;
    activeBoundedStopMode = BoundedStopMode::ACTIVE_BRAKE;
    activeBoundedBrakeStartTime = millis();
    activeBoundedStableSamples = 0;
    motionState = MotionState::BRAKING;
    setActiveCommandState(CommandState::BRAKING);
    activeBoundedTargetReached = true;
    Serial.println("BOUNDED_PREDICTIVE_ACTIVE_BRAKE");
  }

  if (faultPending) {
    // A fault observed during settle re-engages the electrical brake and
    // restarts its hold interval before terminal completion.
    brakeMotors();
    activeBoundedBrakeStartTime = millis();
    activeBoundedFaultPending = true;
    faultReason = reason;
    boundedFaultLatched = true;
    boundedFaultReason = reason;
  }
}

bool enforceBoundedEncoderLimit()
{
  if (!activeBoundedMotion || !motionIsActive()) {
    return false;
  }

  int32_t rightCount;
  int32_t leftCount;
  readMotionCounts(rightCount, leftCount);

  const bool rightLimitReached =
      rightCount >= activeBoundedRightLimitCounts;
  const bool leftLimitReached =
      leftCount >= activeBoundedLeftLimitCounts;
  if (rightLimitReached || leftLimitReached) {
    captureBoundedForwardFirstHardLimitGuard(
        rightCount,
        leftCount,
        rightLimitReached,
        leftLimitReached
    );
    captureBoundedForwardPreBrakeGuard(
        rightCount,
        leftCount,
        BoundedForwardBrakeTrigger::HARD_LIMIT,
        rightCount >= activeBoundedRightBrakeStartCounts,
        leftCount >= activeBoundedLeftBrakeStartCounts,
        rightLimitReached,
        leftLimitReached
    );
    // This absolute per-wheel limit is an emergency guard, not the normal
    // stopping boundary. It is checked even while settling.
    if (!activeBoundedFaultPending) {
      beginBoundedBraking(true, "BOUNDED_DISTANCE_LIMIT");
      return true;
    }
    return false;
  }

  if (activeBoundedBrakeStarted) {
    captureBoundedForwardFirstPostBrakeLoop(rightCount, leftCount);
    if (activeBoundedCorrectionInProgress) {
      const bool targetReached =
          rightCount >= activeBoundedRightTargetCounts ||
          leftCount >= activeBoundedLeftTargetCounts;
      if (targetReached) {
        // A correction may never cross either absolute target. Stop both
        // wheels immediately and send the pulse through the full settle path.
        stopMotors();
        brakeMotors();
        activeBoundedCorrectionInProgress = false;
        activeBoundedBrakeStartTime = millis();
        motionState = MotionState::BRAKING;
        setActiveCommandState(CommandState::BRAKING);
        return true;
      }
    }
    return false;
  }

  if (activeBoundedMotionProfile ==
      BoundedMotionProfile::BOUNDED_MICRO_TURN_V1) {
    const bool rightAtBrakeStart =
        rightCount >= activeBoundedRightBrakeStartCounts;
    const bool leftAtBrakeStart =
        leftCount >= activeBoundedLeftBrakeStartCounts;
    if (rightAtBrakeStart && !activeBoundedRightIndividualBrakeStarted) {
      activeBoundedRightIndividualBrakeStarted = true;
      activeBoundedRightCountAtIndividualBrake = rightCount;
    }
    if (leftAtBrakeStart && !activeBoundedLeftIndividualBrakeStarted) {
      activeBoundedLeftIndividualBrakeStarted = true;
      activeBoundedLeftCountAtIndividualBrake = leftCount;
    }

    CommandRecord* command = activeCommand();
    if (command != nullptr) {
      command->rightIndividualBrakeStarted =
          activeBoundedRightIndividualBrakeStarted;
      command->leftIndividualBrakeStarted =
          activeBoundedLeftIndividualBrakeStarted;
      command->rightCountAtIndividualBrake =
          activeBoundedRightCountAtIndividualBrake;
      command->leftCountAtIndividualBrake =
          activeBoundedLeftCountAtIndividualBrake;
    }

    if (activeBoundedRightIndividualBrakeStarted &&
        activeBoundedLeftIndividualBrakeStarted) {
      beginBoundedTargetSettling();
      return true;
    }

    const int32_t syncErrorCounts = abs(rightCount - leftCount);
    activeBoundedSyncErrorCounts = syncErrorCounts;
    if (command != nullptr) {
      command->syncErrorCounts = syncErrorCounts;
    }
    const bool rightBrake = activeBoundedRightIndividualBrakeStarted;
    const bool leftBrake = activeBoundedLeftIndividualBrakeStarted;
    int rightOutput = TURN_SUSTAIN_PWM;
    int leftOutput = TURN_SUSTAIN_PWM;
    if (syncErrorCounts >= 2) {
      if (rightCount > leftCount) {
        rightOutput = 165;
        leftOutput = 205;
      } else {
        rightOutput = 205;
        leftOutput = 165;
      }
    }
    if (rightBrake) {
      leftOutput = syncErrorCounts >= 2 ? 205 : TURN_SUSTAIN_PWM;
    } else if (leftBrake) {
      rightOutput = syncErrorCounts >= 2 ? 205 : TURN_SUSTAIN_PWM;
    }

    // Recheck each unbraked wheel immediately before its drive output.
    readMotionCounts(rightCount, leftCount);
    if (!rightBrake && rightCount >= activeBoundedRightLimitCounts) {
      beginBoundedBraking(true, "BOUNDED_DISTANCE_LIMIT");
      return true;
    }
    if (!leftBrake && leftCount >= activeBoundedLeftLimitCounts) {
      beginBoundedBraking(true, "BOUNDED_DISTANCE_LIMIT");
      return true;
    }
    applyMicroTurnOutputs(rightOutput, leftOutput, rightBrake, leftBrake);
    return true;
  }

  const float dynamicStopMarginM = currentStopMarginM();
  if (activeBoundedMotionProfile ==
      BoundedMotionProfile::BOUNDED_MICRO_TURN_V1) {
    activeBoundedRightBrakeStartCounts = min(
        activeBoundedRightBrakeStartCounts,
        boundedMicroTurnBrakeStartCounts(
            activeBoundedRightTargetCounts,
            RIGHT_M_PER_COUNT,
            dynamicStopMarginM
        )
    );
    activeBoundedLeftBrakeStartCounts = min(
        activeBoundedLeftBrakeStartCounts,
        boundedMicroTurnBrakeStartCounts(
            activeBoundedLeftTargetCounts,
            LEFT_M_PER_COUNT,
            dynamicStopMarginM
        )
    );
  } else {
    activeBoundedRightBrakeStartCounts = min(
        activeBoundedRightBrakeStartCounts,
        boundedBrakeStartCounts(
            activeBoundedRightTargetCounts,
            RIGHT_M_PER_COUNT,
            dynamicStopMarginM
        )
    );
    activeBoundedLeftBrakeStartCounts = min(
        activeBoundedLeftBrakeStartCounts,
        boundedBrakeStartCounts(
            activeBoundedLeftTargetCounts,
            LEFT_M_PER_COUNT,
            dynamicStopMarginM
        )
    );
  }
  CommandRecord* command = activeCommand();
  if (command != nullptr) {
    command->rightBrakeStartCounts = activeBoundedRightBrakeStartCounts;
    command->leftBrakeStartCounts = activeBoundedLeftBrakeStartCounts;
  }

  const bool predictiveBrake =
      rightCount >= activeBoundedRightBrakeStartCounts ||
      leftCount >= activeBoundedLeftBrakeStartCounts;
  const bool targetReached =
      rightCount >= activeBoundedRightTargetCounts &&
      leftCount >= activeBoundedLeftTargetCounts;
  if (predictiveBrake || targetReached) {
    captureBoundedForwardPreBrakeGuard(
        rightCount,
        leftCount,
        targetReached
            ? BoundedForwardBrakeTrigger::TARGET_REACHED
            : BoundedForwardBrakeTrigger::PREDICTIVE_BRAKE,
        rightCount >= activeBoundedRightBrakeStartCounts,
        leftCount >= activeBoundedLeftBrakeStartCounts,
        rightCount >= activeBoundedRightLimitCounts,
        leftCount >= activeBoundedLeftLimitCounts
    );
    beginBoundedTargetSettling();
    return true;
  }

  return false;
}

bool boundedCorrectionPreconditionsClear(
    int32_t rightCount,
    int32_t leftCount)
{
  return activeBoundedMotion &&
         activeBoundedMotionProfile ==
             BoundedMotionProfile::BOUNDED_MICRO_TURN_V1 &&
         activeCommand() != nullptr &&
         activeBoundedBootSessionId == bootSessionId &&
         activeBoundedCorrectionPulsesUsed <
             activeBoundedMaxCorrectionPulses &&
         rightPwm == 0 && leftPwm == 0 &&
         WiFi.status() == WL_CONNECTED &&
         !stopRequested && !boundedFaultLatched && faultReason.length() == 0 &&
         rightCount < activeBoundedRightTargetCounts &&
         leftCount < activeBoundedLeftTargetCounts;
}

bool startBoundedCorrectionPulse(
    int32_t rightCount,
    int32_t leftCount)
{
  if (!boundedCorrectionPreconditionsClear(rightCount, leftCount)) {
    return false;
  }

  activeBoundedInitialPulseCompleted = true;
  activeBoundedCorrectionPulsesUsed++;
  activeBoundedCountsBeforeCorrectionRight = rightCount;
  activeBoundedCountsBeforeCorrectionLeft = leftCount;
  activeBoundedCorrectionInProgress = true;
  activeBoundedCorrectionPulseStartTime = millis();
  activeBoundedStableSamples = 0;

  CommandRecord* command = activeCommand();
  if (command != nullptr) {
    command->boundedInitialPulseCompleted = true;
    command->boundedCorrectionPulsesUsed = activeBoundedCorrectionPulsesUsed;
    command->boundedCountsBeforeCorrectionRight = rightCount;
    command->boundedCountsBeforeCorrectionLeft = leftCount;
  }

  // Recheck absolute targets immediately before the only correction PWM.
  readMotionCounts(rightCount, leftCount);
  if (!boundedCorrectionPreconditionsClear(rightCount, leftCount)) {
    activeBoundedCorrectionInProgress = false;
    return false;
  }
  setDrivePwm(TURN_SUSTAIN_PWM, TURN_SUSTAIN_PWM);
  motionState = MotionState::CORRECTING;
  setActiveCommandState(CommandState::DRIVING);
  Serial.println("BOUNDED_MICRO_TURN_CORRECTION_START");
  return true;
}

void updateBoundedCorrection(uint32_t now)
{
  int32_t rightCount;
  int32_t leftCount;
  readMotionCounts(rightCount, leftCount);

  if (rightCount < activeBoundedCountsBeforeCorrectionRight ||
      leftCount < activeBoundedCountsBeforeCorrectionLeft) {
    abortMotion("WRONG_DIRECTION");
    return;
  }

  if (now - activeBoundedCorrectionPulseStartTime >=
      BOUNDED_MICRO_TURN_CORRECTION_PULSE_MS) {
    stopMotors();
    brakeMotors();
    activeBoundedCorrectionInProgress = false;
    activeBoundedBrakeStartTime = now;
    activeBoundedStableSamples = 0;
    motionState = MotionState::BRAKING;
    setActiveCommandState(CommandState::BRAKING);
    Serial.println("BOUNDED_MICRO_TURN_CORRECTION_BRAKE");
  }
}

void finishMotion()
{
  stopMotors();
  updateDistancesFromEncoders();

  motionElapsedMs = motionStartTime == 0 ? 0 : millis() - motionStartTime;

  rightSpeedMps = 0.0f;
  leftSpeedMps = 0.0f;
  profileSpeedMps = 0.0f;

  motionState = MotionState::SUCCESS;
  completeActiveCommand(CommandState::SUCCESS);

  Serial.println("MOTION_SUCCESS");
  Serial.print("average_distance_m=");
  Serial.println(averageDistanceM, 4);
  Serial.print("heading_rad=");
  Serial.println(headingRad, 4);
}

void updateBoundedBraking(uint32_t now)
{
  // Keep the verified electrical brake asserted during the first settle hold.
  // There is deliberately no drive-PWM recovery path from BRAKING.
  if (now - activeBoundedBrakeStartTime < BOUNDED_ACTIVE_BRAKE_HOLD_MS) {
    brakeMotors();
    updateDistancesFromEncoders();
    return;
  }

  // Observe the settled encoder counts after releasing the electrical brake.
  // This records remaining mechanical coast; it does not claim that coast is
  // eliminated before a raised-wheel retest.
  stopMotors();
  updateDistancesFromEncoders();
  if (now - activeBoundedBrakeStartTime <
      BOUNDED_ACTIVE_BRAKE_HOLD_MS + BOUNDED_SETTLE_OBSERVE_MS) {
    return;
  }

  readMotionCounts(
      activeBoundedRightFinalCount,
      activeBoundedLeftFinalCount
  );
  if (activeBoundedStableSamples == 0) {
    activeBoundedStableRightCount = activeBoundedRightFinalCount;
    activeBoundedStableLeftCount = activeBoundedLeftFinalCount;
    activeBoundedStableSamples = 1;
  } else if (activeBoundedRightFinalCount == activeBoundedStableRightCount &&
             activeBoundedLeftFinalCount == activeBoundedStableLeftCount) {
    activeBoundedStableSamples++;
  } else {
    activeBoundedStableRightCount = activeBoundedRightFinalCount;
    activeBoundedStableLeftCount = activeBoundedLeftFinalCount;
    activeBoundedStableSamples = 1;
  }
  if (activeBoundedStableSamples < BOUNDED_SETTLED_STABLE_SAMPLES) {
    return;
  }

  CommandRecord* command = activeCommand();
  if (command != nullptr) {
    command->rightFinalCount = activeBoundedRightFinalCount;
    command->leftFinalCount = activeBoundedLeftFinalCount;
    command->rightOvershootCounts =
        activeBoundedRightFinalCount - activeBoundedRightTargetCounts;
    command->leftOvershootCounts =
        activeBoundedLeftFinalCount - activeBoundedLeftTargetCounts;
    command->stopMode = activeBoundedStopMode;
  }

  const bool positiveOvershoot =
      activeBoundedRightFinalCount > activeBoundedRightTargetCounts ||
      activeBoundedLeftFinalCount > activeBoundedLeftTargetCounts;
  const bool completionWithinWindow =
      command != nullptr &&
      activeBoundedRightFinalCount >= command->rightMinSuccessCounts &&
      activeBoundedRightFinalCount <= activeBoundedRightTargetCounts &&
      activeBoundedLeftFinalCount >= command->leftMinSuccessCounts &&
      activeBoundedLeftFinalCount <= activeBoundedLeftTargetCounts;
  activeBoundedTurnFinalAngleRad = headingRad;
  if (command != nullptr) {
    command->boundedTurnFinalAngleRad = headingRad;
  }
  const float targetHeadingRad = activeDirection == MotionDirection::TURN_LEFT
      ? targetTurnAngleRad
      : -targetTurnAngleRad;
  const bool microTurnDirectionValid =
      headingRad * targetHeadingRad >= 0.0f;
  const bool microTurnWheelProgressValid =
      activeBoundedRightFinalCount >= activeBoundedTurnMinWheelProgressCounts &&
      activeBoundedLeftFinalCount >= activeBoundedTurnMinWheelProgressCounts;
  const bool microTurnHeadingValid =
      fabsf(headingRad) >= activeBoundedTurnMinSuccessAngleRad &&
      fabsf(headingRad) <= fabsf(targetHeadingRad);
  const bool microTurnCompletionWithinWindow =
      microTurnDirectionValid &&
      microTurnWheelProgressValid &&
      microTurnHeadingValid;
  const bool completionAccepted =
      activeBoundedMotionProfile == BoundedMotionProfile::BOUNDED_MICRO_TURN_V1
          ? microTurnCompletionWithinWindow
          : completionWithinWindow;
  const float rightProgressRatio = activeBoundedRightTargetCounts > 0
      ? static_cast<float>(activeBoundedRightFinalCount) /
            static_cast<float>(activeBoundedRightTargetCounts)
      : 0.0f;
  const float leftProgressRatio = activeBoundedLeftTargetCounts > 0
      ? static_cast<float>(activeBoundedLeftFinalCount) /
            static_cast<float>(activeBoundedLeftTargetCounts)
      : 0.0f;
  const float actualProgressRatio = constrain(
      min(rightProgressRatio, leftProgressRatio), 0.0f, 1.0f
  );
  if (command != nullptr) {
    command->boundedActualProgressRatio = actualProgressRatio;
    // The initial pulse is complete only after active braking and settled
    // encoder samples have been observed, before terminal evaluation.
    command->boundedInitialPulseCompleted = true;
  }
  activeBoundedInitialPulseCompleted = true;
  if (positiveOvershoot && !activeBoundedFaultPending) {
    // Requested distance/angle is an upper physical bound. Any settled count
    // above target is still a bounded-distance violation.
    activeBoundedFaultPending = true;
    faultReason = "BOUNDED_DISTANCE_LIMIT";
    boundedFaultLatched = true;
    boundedFaultReason = faultReason;
  } else if (!completionAccepted && !activeBoundedFaultPending) {
    if (activeBoundedMotionProfile == BoundedMotionProfile::BOUNDED_MICRO_TURN_V1 &&
        activeBoundedMaxCorrectionPulses > 0 &&
        microTurnDirectionValid && startBoundedCorrectionPulse(
            activeBoundedRightFinalCount,
            activeBoundedLeftFinalCount)) {
      return;
    }
    const bool partialMicroTurn =
        activeBoundedMotionProfile == BoundedMotionProfile::BOUNDED_MICRO_TURN_V1 &&
        activeBoundedRightIndividualBrakeStarted &&
        activeBoundedLeftIndividualBrakeStarted &&
        microTurnDirectionValid &&
        !positiveOvershoot;
    if (partialMicroTurn) {
      if (command != nullptr) {
        command->reobserveRequired = true;
      }
      motionState = MotionState::READY;
      completeActiveCommand(CommandState::PARTIAL_PROGRESS);
      Serial.println("BOUNDED_BRAKE_PARTIAL_PROGRESS");
      return;
    }
    // Predictive braking and the bounded correction budget may still leave an
    // under-travel. Anything below its per-wheel lower window is terminal.
    activeBoundedFaultPending = true;
    faultReason = !microTurnDirectionValid
        ? "WRONG_DIRECTION"
        : "BOUNDED_TARGET_NOT_REACHED";
    boundedFaultLatched = true;
    boundedFaultReason = faultReason;
  }

  if (command != nullptr) {
    command->boundedCorrectionPulsesUsed = activeBoundedCorrectionPulsesUsed;
  }

  motionElapsedMs = motionStartTime == 0 ? 0 : now - motionStartTime;
  rightSpeedMps = 0.0f;
  leftSpeedMps = 0.0f;
  profileSpeedMps = 0.0f;

  if (activeBoundedFaultPending) {
    motionState = MotionState::FAULT;
    completeActiveCommand(CommandState::FAULT);
    Serial.println("BOUNDED_BRAKE_FAULT_SETTLED");
    return;
  }

  // Bounded completion returns the current controller to a new-command-safe
  // READY state. The command record itself remains terminal SUCCESS.
  motionState = MotionState::READY;
  completeActiveCommand(CommandState::SUCCESS);
  Serial.println("BOUNDED_BRAKE_SUCCESS_SETTLED");
}

// ---------------------------------------------------------------------------
// State updates
// ---------------------------------------------------------------------------

void updateStarting(uint32_t now)
{
  const uint32_t elapsed = now - phaseStartTime;

  if (elapsed >= BREAKAWAY_TIMEOUT_MS) {
    abortMotion("BREAKAWAY_NOT_FOUND");
    return;
  }

  int32_t rightCount;
  int32_t leftCount;
  readMotionCounts(rightCount, leftCount);

  if (rightCount < -1 || leftCount < -1) {
    abortMotion("WRONG_DIRECTION");
    return;
  }

  updateDistancesFromEncoders();

  const int32_t breakawayThreshold = boundedBreakawayThresholdCounts();
  if (!rightBreakawayDetected && rightCount >= breakawayThreshold) {
    rightBreakawayDetected = true;
    rightBreakawayPwm = rightPwm;

    Serial.print("RIGHT_BREAKAWAY_PWM=");
    Serial.println(rightBreakawayPwm);
  }

  if (!leftBreakawayDetected && leftCount >= breakawayThreshold) {
    leftBreakawayDetected = true;
    leftBreakawayPwm = leftPwm;

    Serial.print("LEFT_BREAKAWAY_PWM=");
    Serial.println(leftBreakawayPwm);
  }

  if (rightBreakawayDetected && leftBreakawayDetected) {
    transitionToDriving(now);
    return;
  }

  int rampPwm =
      BREAKAWAY_START_PWM +
      static_cast<int>(elapsed / BREAKAWAY_STEP_MS) *
          BREAKAWAY_PWM_STEP;

  rampPwm = constrain(
      rampPwm,
      BREAKAWAY_START_PWM,
      boundedBreakawayPwmCeiling()
  );

  // A wheel that has already broken away moves immediately to its calibrated
  // rolling PWM while the other wheel continues its short adaptive ramp.
  const int commandRight =
      rightBreakawayDetected ? rightSustainPwm() : rampPwm;

  const int commandLeft =
      leftBreakawayDetected ? leftSustainPwm() : rampPwm;

  setDrivePwm(commandRight, commandLeft);
}

float loadedFeedForward(
    float targetSpeed,
    int minimumPwm,
    int pwmAtMaxSpeed)
{
  if (targetSpeed <= 0.005f) {
    return 0.0f;
  }

  const float ratio = constrain(
      targetSpeed / activeMaxProfileSpeedMps(),
      0.0f,
      1.0f
  );

  return minimumPwm +
         (pwmAtMaxSpeed - minimumPwm) * ratio;
}

int calculateWheelPwm(
    float targetSpeed,
    float measuredSpeed,
    float &integral,
    int minimumFeedForward,
    int maxSpeedFeedForward,
    float dt)
{
  if (targetSpeed <= 0.005f) {
    integral = 0.0f;
    return 0;
  }

  // During one command each wheel is driven in only one selected direction.
  // Overspeed is handled by coasting rather than applying reverse torque.
  if (measuredSpeed >
      targetSpeed + OVERSPEED_COAST_MARGIN_M_S) {
    integral *= 0.8f;
    return 0;
  }

  const float speedError = targetSpeed - measuredSpeed;

  integral += speedError * dt;
  integral = constrain(
      integral,
      -INTEGRAL_LIMIT,
      INTEGRAL_LIMIT
  );

  const float feedForward = loadedFeedForward(
      targetSpeed,
      minimumFeedForward,
      maxSpeedFeedForward
  );

  const int output = static_cast<int>(roundf(
      feedForward +
      KP_SPEED * speedError +
      KI_SPEED * integral
  ));

  return constrain(output, 0, MAX_DRIVE_PWM);
}

void updateDriving(uint32_t now)
{
  if (now - previousControlTime < CONTROL_PERIOD_MS) {
    return;
  }

  const float dt =
      (now - previousControlTime) / 1000.0f;

  previousControlTime = now;

  int32_t rightCount;
  int32_t leftCount;
  readMotionCounts(rightCount, leftCount);

  const int32_t deltaRight =
      rightCount - previousRightCount;

  const int32_t deltaLeft =
      leftCount - previousLeftCount;

  lastRightDelta = deltaRight;
  lastLeftDelta = deltaLeft;

  previousRightCount = rightCount;
  previousLeftCount = leftCount;

  if (deltaRight < -1 || deltaLeft < -1) {
    abortMotion("WRONG_DIRECTION");
    return;
  }

  rightDistanceM = rightCount * RIGHT_M_PER_COUNT;
  leftDistanceM = leftCount * LEFT_M_PER_COUNT;

  averageDistanceM =
      (rightDistanceM + leftDistanceM) * 0.5f;

  const float rightPhysicalDistance =
      rightDistanceM * rightPhysicalSign();

  const float leftPhysicalDistance =
      leftDistanceM * leftPhysicalSign();

  headingRad =
      (rightPhysicalDistance - leftPhysicalDistance) / activeTrackWidthM();

  const float remainingDistance =
      targetDistanceM - averageDistanceM;

  if (!activeBoundedMotion &&
      remainingDistance <= currentStopMarginM()) {
    beginCoasting();
    return;
  }

  if (!activeBoundedMotion) {
    const float hardDistanceExtra =
        isTurnCommand()
            ? TURN_HARD_DISTANCE_EXTRA_M
            : HARD_DISTANCE_EXTRA_M;

    if (averageDistanceM >
        targetDistanceM + hardDistanceExtra) {
      abortMotion("DISTANCE_LIMIT");
      return;
    }
  }

  const float brakingSpeed = sqrtf(
      2.0f * DECEL_M_S2 *
      fmaxf(remainingDistance, 0.0f)
  );

  const float maxProfileSpeed = activeMaxProfileSpeedMps();

  const float desiredProfileSpeed =
      fminf(maxProfileSpeed, brakingSpeed);

  if (profileSpeedMps < desiredProfileSpeed) {
    profileSpeedMps = fminf(
        desiredProfileSpeed,
        profileSpeedMps + ACCEL_M_S2 * dt
    );
  } else {
    profileSpeedMps = desiredProfileSpeed;
  }

  const float distanceDifference =
      leftDistanceM - rightDistanceM;

  const float syncCorrection = constrain(
      SYNC_GAIN * distanceDifference,
      -MAX_SYNC_CORRECTION_M_S,
      MAX_SYNC_CORRECTION_M_S
  );

  const float rightTargetSpeed = constrain(
      profileSpeedMps + syncCorrection,
      0.0f,
      maxProfileSpeed
  );

  const float leftTargetSpeed = constrain(
      profileSpeedMps - syncCorrection,
      0.0f,
      maxProfileSpeed
  );

  rightSpeedMps =
      deltaRight * RIGHT_M_PER_COUNT / dt;

  leftSpeedMps =
      deltaLeft * LEFT_M_PER_COUNT / dt;

  rightPwm = calculateWheelPwm(
      rightTargetSpeed,
      rightSpeedMps,
      rightIntegral,
      RIGHT_RUN_PWM_MIN,
      RIGHT_RUN_PWM_AT_MAX,
      dt
  );

  leftPwm = calculateWheelPwm(
      leftTargetSpeed,
      leftSpeedMps,
      leftIntegral,
      LEFT_RUN_PWM_MIN,
      LEFT_RUN_PWM_AT_MAX,
      dt
  );

  // The loaded chassis cannot reliably restart after a controller-requested
  // coast. Preserve the PWM values proven by the loaded sustain test during
  // the whole regulated segment. The fast distance guard below remains
  // responsible for the final cutoff.
  rightPwm = rightPwm < rightSustainPwm()
                 ? rightSustainPwm()
                 : rightPwm;

  leftPwm = leftPwm < leftSustainPwm()
                ? leftSustainPwm()
                : leftPwm;

  const int syncPwmBoost = constrain(
      static_cast<int>(roundf(
          fabsf(rightDistanceM - leftDistanceM) *
          SYNC_PWM_GAIN_PER_M
      )),
      0,
      MAX_SYNC_PWM_BOOST
  );

  if (rightDistanceM > leftDistanceM) {
    leftPwm = constrain(
        leftPwm + syncPwmBoost,
        0,
        MAX_DRIVE_PWM
    );
  } else if (leftDistanceM > rightDistanceM) {
    rightPwm = constrain(
        rightPwm + syncPwmBoost,
        0,
        MAX_DRIVE_PWM
    );
  }

  if (rightTargetSpeed > STALL_CHECK_MIN_TARGET_M_S) {
    rightStallCycles =
        deltaRight <= 0 ? rightStallCycles + 1 : 0;
  } else {
    rightStallCycles = 0;
  }

  if (leftTargetSpeed > STALL_CHECK_MIN_TARGET_M_S) {
    leftStallCycles =
        deltaLeft <= 0 ? leftStallCycles + 1 : 0;
  } else {
    leftStallCycles = 0;
  }

  const bool rightStalled =
      rightStallCycles >= STALL_CYCLES_LIMIT;

  const bool leftStalled =
      leftStallCycles >= STALL_CYCLES_LIMIT;

  if (rightStalled && leftStalled) {
    abortMotion("BOTH_ENCODERS_OR_STALL");
    return;
  }

  if (rightStalled) {
    abortMotion("RIGHT_ENCODER_OR_STALL");
    return;
  }

  if (leftStalled) {
    abortMotion("LEFT_ENCODER_OR_STALL");
    return;
  }

  if (activeBoundedMotion && activeBoundedMotionProfile ==
      BoundedMotionProfile::BOUNDED_MICRO_TURN_V1) {
    // A drive write is allowed only while both wheels are still unbraked.
    // The fast encoder guard above owns the transition to individual brake.
    int32_t outputRightCount;
    int32_t outputLeftCount;
    readMotionCounts(outputRightCount, outputLeftCount);
    if (activeBoundedRightIndividualBrakeStarted ||
        activeBoundedLeftIndividualBrakeStarted ||
        outputRightCount >= activeBoundedRightBrakeStartCounts ||
        outputLeftCount >= activeBoundedLeftBrakeStartCounts ||
        outputRightCount >= activeBoundedRightLimitCounts ||
        outputLeftCount >= activeBoundedLeftLimitCounts) {
      return;
    }
    const int32_t syncErrorCounts = abs(outputRightCount - outputLeftCount);
    activeBoundedSyncErrorCounts = syncErrorCounts;
    int rightOutput = TURN_SUSTAIN_PWM;
    int leftOutput = TURN_SUSTAIN_PWM;
    if (syncErrorCounts >= 2) {
      if (outputRightCount > outputLeftCount) {
        rightOutput = 165;
        leftOutput = 205;
      } else {
        rightOutput = 205;
        leftOutput = 165;
      }
    }
    CommandRecord* command = activeCommand();
    if (command != nullptr) {
      command->syncErrorCounts = syncErrorCounts;
    }
    applyMicroTurnOutputs(rightOutput, leftOutput, false, false);
  } else {
    setDrivePwm(rightPwm, leftPwm);
  }
}

void updateMotion()
{
  if (!motionIsActive()) {
    stopMotors();
    return;
  }

  if (stopRequested) {
    stopRequested = false;
    abortMotion("REMOTE_STOP");
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    abortMotion("WIFI_LOST");
    return;
  }

  // Run before the 100 ms PI cadence in STARTING, DRIVING and COASTING.
  // It owns the bounded command's terminal encoder cutoff.
  if (enforceBoundedEncoderLimit()) {
    return;
  }

  const uint32_t now = millis();

  if (!activeBoundedMotion && motionState == MotionState::DRIVING) {
    updateDistancesFromEncoders();

    if (targetDistanceM - averageDistanceM <= currentStopMarginM()) {
      beginCoasting();
      return;
    }
  }

  const uint32_t activeTimeoutMs = activeBoundedMotion
      ? activeBoundedTimeoutMs
      : motionTimeoutMs;
  if (now - motionStartTime >= activeTimeoutMs) {
    abortMotion("TIMEOUT");
    return;
  }

  switch (motionState) {
    case MotionState::STARTING:
      updateStarting(now);
      break;

    case MotionState::DRIVING:
      updateDriving(now);
      break;

    case MotionState::CORRECTING:
      updateBoundedCorrection(now);
      break;

    case MotionState::BRAKING:
      updateBoundedBraking(now);
      break;

    case MotionState::COASTING:
      if (now - coastStartTime < ACTIVE_BRAKE_TIME_MS) {
        brakeMotors();
      } else {
        stopMotors();
      }

      updateDistancesFromEncoders();

      if (now - coastStartTime >= COAST_TIME_MS) {
        finishMotion();
      }
      break;

    default:
      stopMotors();
      break;
  }
}

// ---------------------------------------------------------------------------
// Autonomous square sequence
// ---------------------------------------------------------------------------

MotionDirection squareDirectionForSegment(uint8_t segmentIndex)
{
  return segmentIndex % 2 == 0
             ? MotionDirection::FORWARD
             : MotionDirection::TURN_LEFT;
}

void accumulateSquarePose()
{
  const float rightPhysicalDistance =
      rightDistanceM * rightPhysicalSign();

  const float leftPhysicalDistance =
      leftDistanceM * leftPhysicalSign();

  const float centerDistance =
      (rightPhysicalDistance + leftPhysicalDistance) * 0.5f;

  const float headingChange =
      (rightPhysicalDistance - leftPhysicalDistance) / activeTrackWidthM();

  const float midpointHeading =
      squarePoseHeadingRad + headingChange * 0.5f;

  squarePoseXM += centerDistance * cosf(midpointHeading);
  squarePoseYM += centerDistance * sinf(midpointHeading);
  squarePoseHeadingRad += headingChange;
}

void failSquareSequence(const String &reason)
{
  stopMotors();
  squareSegmentActive = false;
  squareFaultReason = reason;
  squareElapsedMs =
      squareStartTime == 0 ? 0 : millis() - squareStartTime;
  squareState = SquareState::FAULT;

  Serial.print("SQUARE_FAULT=");
  Serial.println(reason);
}

void startSquareSequence()
{
  stopMotors();

  squareState = SquareState::RUNNING;
  squareSegmentActive = true;
  squareSegmentsCompleted = 0;
  squareStartTime = millis();
  squareElapsedMs = 0;
  squarePauseStartTime = 0;
  squarePoseXM = 0.0f;
  squarePoseYM = 0.0f;
  squarePoseHeadingRad = 0.0f;
  squareFaultReason = "";

  Serial.println("SQUARE_START segment=1/8 action=FORWARD");
  startMotion(
      MotionDirection::FORWARD,
      FORWARD_TARGET_DISTANCE_M,
      FORWARD_TIMEOUT_MS,
      false,
      0.0f
  );
}

void updateSquareSequence()
{
  if (!squareIsActive()) {
    return;
  }

  const uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (motionIsActive()) {
      abortMotion("WIFI_LOST");
    }
    failSquareSequence("WIFI_LOST");
    return;
  }

  if (now - squareStartTime >= SQUARE_TOTAL_TIMEOUT_MS) {
    if (motionIsActive()) {
      abortMotion("SQUARE_TIMEOUT");
    }
    failSquareSequence("SQUARE_TIMEOUT");
    return;
  }

  if (motionState == MotionState::FAULT) {
    failSquareSequence(faultReason);
    return;
  }

  if (squareSegmentActive && motionState == MotionState::SUCCESS) {
    accumulateSquarePose();
    squareSegmentsCompleted++;
    squareSegmentActive = false;
    squarePauseStartTime = now;

    Serial.print("SQUARE_SEGMENT_COMPLETE=");
    Serial.println(squareSegmentsCompleted);

    if (squareSegmentsCompleted >= SQUARE_SEGMENT_COUNT) {
      squareElapsedMs = now - squareStartTime;
      squareState = SquareState::SUCCESS;

      Serial.println("SQUARE_SUCCESS");
      return;
    }
  }

  if (!squareSegmentActive &&
      now - squarePauseStartTime >= SQUARE_SETTLE_TIME_MS) {
    const MotionDirection nextDirection =
        squareDirectionForSegment(squareSegmentsCompleted);

    squareSegmentActive = true;

    Serial.print("SQUARE_START segment=");
    Serial.print(squareSegmentsCompleted + 1);
    Serial.print("/8 action=");

    activeDirection = nextDirection;
    Serial.println(directionName());

    const bool turning =
        nextDirection == MotionDirection::TURN_LEFT ||
        nextDirection == MotionDirection::TURN_RIGHT;
    startMotion(
        nextDirection,
        turning ? TURN_SQUARE_TARGET_WHEEL_DISTANCE_M
                : FORWARD_TARGET_DISTANCE_M,
        turning ? TURN_TIMEOUT_MS : FORWARD_TIMEOUT_MS,
        turning,
        turning ? TURN_TARGET_ANGLE_RAD : 0.0f
    );
  }
}

// ---------------------------------------------------------------------------
// HTTP telemetry and UI
// ---------------------------------------------------------------------------

void appendJsonNullableString(String &json, const String *value)
{
  if (value == nullptr) {
    json += "null";
    return;
  }
  json += "\"";
  json += *value;
  json += "\"";
}

void appendJsonNullableFloat(String &json, bool present, float value)
{
  if (!present) {
    json += "null";
    return;
  }
  json += String(value, 4);
}

void appendJsonNullableMillis(String &json, uint32_t value)
{
  if (value == 0) {
    json += "null";
    return;
  }
  json += String(value);
}

void appendJsonNullableInt(String &json, bool present, int32_t value)
{
  if (!present) {
    json += "null";
    return;
  }
  json += String(value);
}

void appendBoundedForwardBrakeDiagnostics(
    String &json,
    const CommandRecord* bounded)
{
  if (!activeBoundedForwardCommand(bounded)) {
    json += "null";
    return;
  }

  json += "{\"pre_brake_guard\":";
  if (!bounded->forwardPreBrakeGuardCaptured) {
    json += "null";
  } else {
    json += "{\"timestamp_ms\":";
    json += String(bounded->forwardPreBrakeGuardTimestampMs);
    json += ",\"controller_state\":\"";
    json += motionStateName(bounded->forwardPreBrakeGuardState);
    json += "\",\"right_count\":";
    json += String(bounded->forwardPreBrakeRightCount);
    json += ",\"left_count\":";
    json += String(bounded->forwardPreBrakeLeftCount);
    json += ",\"right_brake_start_count\":";
    json += String(bounded->forwardPreBrakeRightBrakeStartCounts);
    json += ",\"left_brake_start_count\":";
    json += String(bounded->forwardPreBrakeLeftBrakeStartCounts);
    json += ",\"right_hard_limit_count\":";
    json += String(bounded->forwardPreBrakeRightLimitCounts);
    json += ",\"left_hard_limit_count\":";
    json += String(bounded->forwardPreBrakeLeftLimitCounts);
    json += ",\"trigger_reason\":\"";
    json += boundedForwardBrakeTriggerName(bounded->forwardPreBrakeTrigger);
    json += "\",\"right_met_brake_threshold\":";
    json += bounded->forwardPreBrakeRightAtBrakeThreshold ? "true" : "false";
    json += ",\"left_met_brake_threshold\":";
    json += bounded->forwardPreBrakeLeftAtBrakeThreshold ? "true" : "false";
    json += ",\"right_met_hard_limit\":";
    json += bounded->forwardPreBrakeRightAtHardLimit ? "true" : "false";
    json += ",\"left_met_hard_limit\":";
    json += bounded->forwardPreBrakeLeftAtHardLimit ? "true" : "false";
    json += "}";
  }

  json += ",\"brake_command\":";
  if (!bounded->forwardBrakeCommandCaptured) {
    json += "null";
  } else {
    json += "{\"timestamp_ms\":";
    json += String(bounded->forwardBrakeCommandTimestampMs);
    json += ",\"controller_state\":\"";
    json += motionStateName(bounded->forwardBrakeCommandState);
    json += "\",\"right_count\":";
    json += String(bounded->forwardBrakeCommandRightCount);
    json += ",\"left_count\":";
    json += String(bounded->forwardBrakeCommandLeftCount);
    json += ",\"right_pwm\":";
    json += String(bounded->forwardBrakeCommandRightPwm);
    json += ",\"left_pwm\":";
    json += String(bounded->forwardBrakeCommandLeftPwm);
    json += "}";
  }

  json += ",\"first_post_brake_loop\":";
  if (!bounded->forwardFirstPostBrakeLoopCaptured) {
    json += "null";
  } else {
    json += "{\"timestamp_ms\":";
    json += String(bounded->forwardFirstPostBrakeLoopTimestampMs);
    json += ",\"controller_state\":\"";
    json += motionStateName(bounded->forwardFirstPostBrakeLoopState);
    json += "\",\"right_count\":";
    json += String(bounded->forwardFirstPostBrakeLoopRightCount);
    json += ",\"left_count\":";
    json += String(bounded->forwardFirstPostBrakeLoopLeftCount);
    json += ",\"right_pwm\":";
    json += String(bounded->forwardFirstPostBrakeLoopRightPwm);
    json += ",\"left_pwm\":";
    json += String(bounded->forwardFirstPostBrakeLoopLeftPwm);
    json += "}";
  }

  json += ",\"first_hard_limit_guard\":";
  if (!bounded->forwardFirstHardLimitGuardCaptured) {
    json += "null";
  } else {
    json += "{\"timestamp_ms\":";
    json += String(bounded->forwardFirstHardLimitGuardTimestampMs);
    json += ",\"right_count\":";
    json += String(bounded->forwardFirstHardLimitGuardRightCount);
    json += ",\"left_count\":";
    json += String(bounded->forwardFirstHardLimitGuardLeftCount);
    json += ",\"right_exceeded\":";
    json += bounded->forwardFirstHardLimitGuardRightExceeded ? "true" : "false";
    json += ",\"left_exceeded\":";
    json += bounded->forwardFirstHardLimitGuardLeftExceeded ? "true" : "false";
    json += ",\"already_braking_or_settling\":";
    json += bounded->forwardFirstHardLimitGuardAlreadyBraking ? "true" : "false";
    json += "}";
  }
  json += "}";
}

String buildStatusJson()
{
  updateDistancesFromEncoders();

  int32_t rightCount;
  int32_t leftCount;
  readMotionCounts(rightCount, leftCount);

  const uint32_t elapsedMs = motionIsActive()
                                 ? millis() - motionStartTime
                                 : motionElapsedMs;

  const uint32_t currentSquareElapsedMs =
      squareIsActive()
          ? millis() - squareStartTime
          : squareElapsedMs;

  const float squareClosureErrorM =
      sqrtf(squarePoseXM * squarePoseXM +
            squarePoseYM * squarePoseYM);

  String json;
  json.reserve(1800);

  json += "{";

  json += "\"firmware\":\"";
  json += FIRMWARE_ID;
  json += "\",";

  json += "\"motion_api_version\":\"v2_bounded_motion_api\",";

  json += "\"boot_session_id\":\"";
  json += bootSessionId;
  json += "\",";

  json += "\"bounded_fault_latched\":";
  json += boundedFaultLatched ? "true," : "false,";

  json += "\"bounded_fault_reason\":";
  if (boundedFaultLatched) {
    appendJsonNullableString(json, &boundedFaultReason);
  } else {
    json += "null";
  }
  json += ",";

  json += "\"state\":\"";
  json += stateName();
  json += "\",";

  json += "\"direction\":\"";
  json += directionName();
  json += "\",";

  CommandRecord* active = activeCommand();
  CommandRecord* last = lastCommand();
  const String* activeId =
      active == nullptr ? nullptr : &active->commandId;
  const String* lastId =
      last == nullptr ? nullptr : &last->commandId;
  const String* lastEndpoint =
      last == nullptr ? nullptr : &last->endpoint;

  json += "\"active_command_id\":";
  appendJsonNullableString(json, activeId);
  json += ",";
  json += "\"last_command_id\":";
  appendJsonNullableString(json, lastId);
  json += ",";
  json += "\"last_command_endpoint\":";
  appendJsonNullableString(json, lastEndpoint);
  json += ",";
  json += "\"last_command_state\":";
  if (last == nullptr) {
    json += "null";
  } else {
    json += "\"";
    json += commandStateName(last->state);
    json += "\"";
  }
  json += ",";
  json += "\"last_command_distance_m\":";
  appendJsonNullableFloat(
      json,
      last != nullptr && last->hasDistance,
      last == nullptr ? 0.0f : last->distanceM
  );
  json += ",";
  json += "\"last_command_angle_deg\":";
  appendJsonNullableFloat(
      json,
      last != nullptr && last->hasAngle,
      last == nullptr ? 0.0f : last->angleDeg
  );
  json += ",";
  const CommandRecord* timing = active == nullptr ? last : active;
  json += "\"command_started_at_ms\":";
  appendJsonNullableMillis(
      json,
      timing == nullptr ? 0 : timing->startedAtMs
  );
  json += ",";
  json += "\"command_completed_at_ms\":";
  appendJsonNullableMillis(
      json,
      timing == nullptr ? 0 : timing->completedAtMs
  );
  json += ",";

  const bool hasBoundedTelemetry =
      timing != nullptr && timing->boundedKind != BoundedMotionKind::NONE;
  const CommandRecord* bounded = hasBoundedTelemetry ? timing : nullptr;
  json += "\"bounded_forward_brake_diagnostics\":";
  appendBoundedForwardBrakeDiagnostics(json, bounded);
  json += ",";
  json += "\"bounded_motion_profile\":";
  if (bounded == nullptr) {
    json += "null";
  } else {
    json += "\"";
    json += boundedMotionProfileName(bounded->boundedMotionProfile);
    json += "\"";
  }
  json += ",";
  json += "\"bounded_turn_brake_reserve_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->turnBrakeReserveCounts > 0,
      bounded == nullptr ? 0 : bounded->turnBrakeReserveCounts
  );
  json += ",";
  json += "\"bounded_right_individual_brake_started\":";
  if (bounded == nullptr) {
    json += "null";
  } else {
    json += bounded->rightIndividualBrakeStarted ? "true" : "false";
  }
  json += ",";
  json += "\"bounded_left_individual_brake_started\":";
  if (bounded == nullptr) {
    json += "null";
  } else {
    json += bounded->leftIndividualBrakeStarted ? "true" : "false";
  }
  json += ",";
  json += "\"bounded_right_count_at_individual_brake\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->rightIndividualBrakeStarted,
      bounded == nullptr ? 0 : bounded->rightCountAtIndividualBrake
  );
  json += ",";
  json += "\"bounded_left_count_at_individual_brake\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->leftIndividualBrakeStarted,
      bounded == nullptr ? 0 : bounded->leftCountAtIndividualBrake
  );
  json += ",";
  json += "\"bounded_right_last_applied_pwm\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->rightLastAppliedPwm
  );
  json += ",";
  json += "\"bounded_left_last_applied_pwm\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->leftLastAppliedPwm
  );
  json += ",";
  json += "\"bounded_sync_error_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->syncErrorCounts
  );
  json += ",";
  json += "\"bounded_actual_progress_ratio\":";
  appendJsonNullableFloat(
      json,
      bounded != nullptr && bounded->completedAtMs != 0,
      bounded == nullptr ? 0.0f : bounded->boundedActualProgressRatio
  );
  json += ",";
  json += "\"reobserve_required\":";
  json += bounded != nullptr && bounded->reobserveRequired ? "true" : "false";
  json += ",";
  json += "\"bounded_correction_pulses_enabled\":false,";
  json += "\"bounded_initial_pulse_completed\":";
  if (bounded == nullptr) {
    json += "null";
  } else {
    json += bounded->boundedInitialPulseCompleted ? "true" : "false";
  }
  json += ",";
  json += "\"bounded_correction_pulses_used\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->boundedCorrectionPulsesUsed
  );
  json += ",";
  json += "\"bounded_max_correction_pulses\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->boundedMaxCorrectionPulses
  );
  json += ",";
  json += "\"bounded_counts_before_correction_right\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->boundedCorrectionPulsesUsed > 0,
      bounded == nullptr ? 0 : bounded->boundedCountsBeforeCorrectionRight
  );
  json += ",";
  json += "\"bounded_counts_before_correction_left\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->boundedCorrectionPulsesUsed > 0,
      bounded == nullptr ? 0 : bounded->boundedCountsBeforeCorrectionLeft
  );
  json += ",";
  json += "\"bounded_right_target_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->rightTargetCounts
  );
  json += ",";
  json += "\"bounded_left_target_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->leftTargetCounts
  );
  json += ",";
  json += "\"bounded_right_completion_tolerance_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->rightCompletionToleranceCounts
  );
  json += ",";
  json += "\"bounded_left_completion_tolerance_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->leftCompletionToleranceCounts
  );
  json += ",";
  json += "\"bounded_right_min_success_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->rightMinSuccessCounts
  );
  json += ",";
  json += "\"bounded_left_min_success_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->leftMinSuccessCounts
  );
  json += ",";
  json += "\"bounded_turn_min_success_angle_rad\":";
  appendJsonNullableFloat(
      json,
      bounded != nullptr && bounded->boundedMotionProfile ==
          BoundedMotionProfile::BOUNDED_MICRO_TURN_V1,
      bounded == nullptr ? 0.0f : bounded->boundedTurnMinSuccessAngleRad
  );
  json += ",";
  json += "\"bounded_turn_final_angle_rad\":";
  appendJsonNullableFloat(
      json,
      bounded != nullptr && bounded->boundedMotionProfile ==
          BoundedMotionProfile::BOUNDED_MICRO_TURN_V1 &&
          bounded->completedAtMs != 0,
      bounded == nullptr ? 0.0f : bounded->boundedTurnFinalAngleRad
  );
  json += ",";
  json += "\"bounded_turn_min_wheel_progress_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->boundedMotionProfile ==
          BoundedMotionProfile::BOUNDED_MICRO_TURN_V1,
      bounded == nullptr ? 0 : bounded->boundedTurnMinWheelProgressCounts
  );
  json += ",";
  json += "\"bounded_right_brake_start_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->rightBrakeStartCounts
  );
  json += ",";
  json += "\"bounded_left_brake_start_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr,
      bounded == nullptr ? 0 : bounded->leftBrakeStartCounts
  );
  json += ",";
  json += "\"bounded_right_count_at_brake_start\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->stopMode != BoundedStopMode::NONE,
      bounded == nullptr ? 0 : bounded->rightCountAtBrakeStart
  );
  json += ",";
  json += "\"bounded_left_count_at_brake_start\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->stopMode != BoundedStopMode::NONE,
      bounded == nullptr ? 0 : bounded->leftCountAtBrakeStart
  );
  json += ",";
  json += "\"bounded_right_final_settled_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->completedAtMs != 0,
      bounded == nullptr ? 0 : bounded->rightFinalCount
  );
  json += ",";
  json += "\"bounded_left_final_settled_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->completedAtMs != 0,
      bounded == nullptr ? 0 : bounded->leftFinalCount
  );
  json += ",";
  json += "\"bounded_right_overshoot_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->completedAtMs != 0,
      bounded == nullptr ? 0 : bounded->rightOvershootCounts
  );
  json += ",";
  json += "\"bounded_left_overshoot_counts\":";
  appendJsonNullableInt(
      json,
      bounded != nullptr && bounded->completedAtMs != 0,
      bounded == nullptr ? 0 : bounded->leftOvershootCounts
  );
  json += ",";
  json += "\"bounded_stop_mode\":";
  if (bounded == nullptr) {
    json += "null";
  } else {
    json += "\"";
    json += boundedStopModeName(bounded->stopMode);
    json += "\"";
  }
  json += ",";

  json += "\"wifi\":";
  json += (WiFi.status() == WL_CONNECTED ? "true," : "false,");

  json += "\"ip\":\"";
  json += WiFi.localIP().toString();
  json += "\",";

  json += "\"square_state\":\"";
  json += squareStateName();
  json += "\",";

  json += "\"square_segments_completed\":";
  json += String(squareSegmentsCompleted);
  json += ",";

  json += "\"square_segments_total\":";
  json += String(SQUARE_SEGMENT_COUNT);
  json += ",";

  json += "\"square_current_action\":\"";
  json += squareCurrentActionName();
  json += "\",";

  json += "\"square_pose_x_m\":";
  json += String(squarePoseXM, 4);
  json += ",";

  json += "\"square_pose_y_m\":";
  json += String(squarePoseYM, 4);
  json += ",";

  json += "\"square_heading_rad\":";
  json += String(squarePoseHeadingRad, 4);
  json += ",";

  json += "\"square_heading_error_rad\":";
  json += String(normalizeAngleRad(squarePoseHeadingRad), 4);
  json += ",";

  json += "\"square_closure_error_m\":";
  json += String(squareClosureErrorM, 4);
  json += ",";

  json += "\"square_elapsed_ms\":";
  json += String(currentSquareElapsedMs);
  json += ",";

  json += "\"square_fault\":\"";
  json += squareFaultReason;
  json += "\",";

  json += "\"right_count\":";
  json += String(rightCount);
  json += ",";

  json += "\"left_count\":";
  json += String(leftCount);
  json += ",";

  json += "\"right_distance_m\":";
  json += String(rightDistanceM, 4);
  json += ",";

  json += "\"left_distance_m\":";
  json += String(leftDistanceM, 4);
  json += ",";

  json += "\"average_distance_m\":";
  json += String(averageDistanceM, 4);
  json += ",";

  json += "\"target_distance_m\":";
  json += String(targetDistanceM, 3);
  json += ",";

  json += "\"right_speed_m_s\":";
  json += String(rightSpeedMps, 3);
  json += ",";

  json += "\"left_speed_m_s\":";
  json += String(leftSpeedMps, 3);
  json += ",";

  json += "\"profile_speed_m_s\":";
  json += String(profileSpeedMps, 3);
  json += ",";

  json += "\"turn_sustain_pwm\":";
  json += String(isTurnCommand() ? TURN_SUSTAIN_PWM : 0);
  json += ",";

  json += "\"geometric_track_width_m\":";
  json += String(TRACK_WIDTH_M, 4);
  json += ",";

  json += "\"effective_turn_track_width_m\":";
  json += String(TURN_EFFECTIVE_TRACK_WIDTH_M, 4);
  json += ",";

  json += "\"active_turn_track_width_m\":";
  json += String(isTurnCommand() ? activeTrackWidthM() : 0.0f, 4);
  json += ",";

  json += "\"right_pwm\":";
  json += String(rightPwm);
  json += ",";

  json += "\"left_pwm\":";
  json += String(leftPwm);
  json += ",";

  json += "\"heading_rad\":";
  json += String(headingRad, 4);
  json += ",";

  json += "\"target_turn_angle_rad\":";
  json += String(isTurnCommand()
                     ? (activeDirection == MotionDirection::TURN_LEFT
                            ? targetTurnAngleRad
                            : -targetTurnAngleRad)
                     : 0.0f, 4);
  json += ",";

  json += "\"turn_error_rad\":";
  json += String(isTurnCommand()
                     ? ((activeDirection == MotionDirection::TURN_LEFT
                             ? targetTurnAngleRad
                             : -targetTurnAngleRad) - headingRad)
                     : 0.0f, 4);
  json += ",";

  json += "\"right_breakaway_pwm\":";
  json += String(rightBreakawayPwm);
  json += ",";

  json += "\"left_breakaway_pwm\":";
  json += String(leftBreakawayPwm);
  json += ",";

  json += "\"right_count_at_stop\":";
  json += String(rightCountAtStop);
  json += ",";

  json += "\"left_count_at_stop\":";
  json += String(leftCountAtStop);
  json += ",";

  json += "\"right_pwm_at_stop\":";
  json += String(rightPwmAtStop);
  json += ",";

  json += "\"left_pwm_at_stop\":";
  json += String(leftPwmAtStop);
  json += ",";

  json += "\"stop_margin_at_stop_m\":";
  json += String(stopMarginAtStopM, 4);
  json += ",";

  json += "\"last_right_delta\":";
  json += String(lastRightDelta);
  json += ",";

  json += "\"last_left_delta\":";
  json += String(lastLeftDelta);
  json += ",";

  json += "\"right_stall_cycles\":";
  json += String(rightStallCycles);
  json += ",";

  json += "\"left_stall_cycles\":";
  json += String(leftStallCycles);
  json += ",";

  json += "\"elapsed_ms\":";
  json += String(elapsedMs);
  json += ",";

  json += "\"fault\":\"";
  json += faultReason;
  json += "\"";

  json += "}";

  return json;
}

void handleRoot()
{
  server.send(
      200,
      "text/html; charset=utf-8",
      R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>SIE Base Control</title>
  <style>
    :root { color-scheme: dark; }
    body {
      font-family: system-ui, sans-serif;
      max-width: 720px;
      margin: 24px auto;
      padding: 0 14px;
      background: #101214;
      color: #edf1f4;
    }
    .panel {
      background: #1b1f23;
      border: 1px solid #30363d;
      border-radius: 12px;
      padding: 16px;
      margin-bottom: 14px;
    }
    button {
      width: 100%;
      padding: 18px;
      margin: 7px 0;
      font-size: 19px;
      font-weight: 700;
      border: 0;
      border-radius: 10px;
      cursor: pointer;
    }
    #forward { background: #238636; color: white; }
    #reverse { background: #1f6feb; color: white; }
    #left, #right { background: #8957e5; color: white; }
    #square { background: #d29922; color: #101214; }
    #stop { background: #da3633; color: white; }
    #state { font-size: 28px; font-weight: 750; }
    pre {
      white-space: pre-wrap;
      word-break: break-word;
      margin: 0;
      font-size: 14px;
    }
    .warning { color: #f2cc60; }
  </style>
</head>
<body>
  <h1>SIE Base</h1>

  <div class="panel">
    <div id="state">Loading...</div>
    <p>Квадрат: 4 × (0,50 м вперёд + 90° влево).</p>
    <p class="warning">
      Для SQUARE освободите площадку не менее 1,2 × 1,2 м и держите STOP открытым.
    </p>
  </div>

  <button id="forward" onclick="requestMotion('/start')">
    FORWARD 0.50 m
  </button>

  <button id="reverse" onclick="requestMotion('/reverse')">
    REVERSE 0.20 m
  </button>

  <button id="left" onclick="requestMotion('/turn-left')">
    TURN LEFT 90 deg
  </button>

  <button id="right" onclick="requestMotion('/turn-right')">
    TURN RIGHT 90 deg
  </button>

  <button id="square" onclick="requestMotion('/square')">
    SQUARE 0.50 m × 4
  </button>

  <button id="stop" onclick="stopMotion()">
    EMERGENCY STOP
  </button>

  <div class="panel">
    <pre id="status">Loading...</pre>
  </div>

  <script>
    async function requestMotion(path) {
      const message = path === "/square"
        ? "Свободна площадка 1,2 × 1,2 м, STOP открыт, людей и проводов рядом нет?"
        : "Зона свободна и кнопка STOP доступна?";

      if (!confirm(message)) return;

      const response = await fetch(path, { method: "POST" });
      if (!response.ok) alert(await response.text());
      await updateStatus();
    }

    async function stopMotion() {
      await fetch("/stop", { method: "POST" });
      await updateStatus();
    }

    async function updateStatus() {
      try {
        const response = await fetch("/status?t=" + Date.now());
        const data = await response.json();

        document.getElementById("state").textContent =
          data.square_state === "RUNNING"
            ? "SQUARE " + data.square_segments_completed + "/8 · " +
                data.square_current_action
            : data.square_state === "SUCCESS"
                ? "SQUARE SUCCESS"
                : data.state;
        document.getElementById("status").textContent =
          JSON.stringify(data, null, 2);
      } catch (error) {
        document.getElementById("state").textContent = "CONNECTION LOST";
        document.getElementById("status").textContent = String(error);
      }
    }

    setInterval(updateStatus, 200);
    updateStatus();
  </script>
</body>
</html>
)HTML"
  );
}

bool motionOrSquareIsBusy()
{
  return motionIsActive() || startRequested ||
         squareIsActive() || squareRequested;
}

bool hasExactlyArguments(const char* first, const char* second)
{
  return server.args() == 2 && server.hasArg(first) && server.hasArg(second);
}

bool hasExactlyArguments(
    const char* first,
    const char* second,
    const char* third)
{
  return server.args() == 3 && server.hasArg(first) &&
         server.hasArg(second) && server.hasArg(third);
}

void sendJson(int status, const String &body)
{
  server.send(status, "application/json", body);
}

void queueLegacyMotionRequest(MotionDirection direction)
{
  if (boundedFaultLatched) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"BOUNDED_FAULT_LATCHED\"}");
    return;
  }

  if (motionOrSquareIsBusy()) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"busy\"}");
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    sendJson(503, "{\"accepted\":false,\"reason\":\"wifi\"}");
    return;
  }

  requestedDirection = direction;
  if (direction == MotionDirection::FORWARD) {
    requestedTargetDistanceM = FORWARD_TARGET_DISTANCE_M;
    requestedTimeoutMs = FORWARD_TIMEOUT_MS;
    requestedTurnUsesEffectiveTrack = false;
    requestedTurnAngleRad = 0.0f;
  } else if (direction == MotionDirection::REVERSE) {
    requestedTargetDistanceM = REVERSE_TARGET_DISTANCE_M;
    requestedTimeoutMs = REVERSE_TIMEOUT_MS;
    requestedTurnUsesEffectiveTrack = false;
    requestedTurnAngleRad = 0.0f;
  } else {
    requestedTargetDistanceM = TURN_SINGLE_TARGET_WHEEL_DISTANCE_M;
    requestedTimeoutMs = TURN_TIMEOUT_MS;
    requestedTurnUsesEffectiveTrack = false;
    requestedTurnAngleRad = TURN_TARGET_ANGLE_RAD;
  }
  startRequested = true;
  sendJson(202, "{\"accepted\":true}");
}

void handleStart()
{
  queueLegacyMotionRequest(MotionDirection::FORWARD);
}

void handleReverse()
{
  queueLegacyMotionRequest(MotionDirection::REVERSE);
}

bool matchingCommand(
    const CommandRecord &record,
    const String &endpoint,
    bool hasDistance,
    float distanceM,
    bool hasAngle,
    float angleDeg)
{
  return record.endpoint == endpoint &&
         record.hasDistance == hasDistance &&
         record.hasAngle == hasAngle &&
         (!hasDistance || fabsf(record.distanceM - distanceM) <= 0.000001f) &&
         (!hasAngle || fabsf(record.angleDeg - angleDeg) <= 0.000001f);
}

void sendDuplicateCommand(const CommandRecord &record)
{
  String body = "{\"accepted\":false,\"duplicate\":true,\"command_id\":\"";
  body += record.commandId;
  body += "\",\"command_state\":\"";
  body += commandStateName(record.state);
  body += "\"}";
  sendJson(200, body);
}

void queueBoundedMotionRequest(
    const String &endpoint,
    MotionDirection direction,
    const String &requestBootSessionId,
    const String &commandId,
    bool hasDistance,
    float distanceM,
    bool hasAngle,
    float angleDeg)
{
  if (requestBootSessionId != bootSessionId) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"BOOT_SESSION_MISMATCH\"}");
    return;
  }

  if (!commandIdIsValid(commandId)) {
    sendJson(400, "{\"accepted\":false,\"reason\":\"INVALID_COMMAND_ID\"}");
    return;
  }

  const int8_t existingIndex = findCommand(commandId);
  if (existingIndex >= 0) {
    const CommandRecord &existing = commandHistory[existingIndex];
    if (matchingCommand(
            existing,
            endpoint,
            hasDistance,
            distanceM,
            hasAngle,
            angleDeg)) {
      sendDuplicateCommand(existing);
    } else {
      sendJson(409, "{\"accepted\":false,\"reason\":\"COMMAND_ID_CONFLICT\"}");
    }
    return;
  }

  if (boundedFaultLatched) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"BOUNDED_FAULT_LATCHED\"}");
    return;
  }

  // A bounded request may follow fault acknowledgement only after the handler
  // has restored the controller from FAULT to READY.
  if (motionState == MotionState::FAULT || faultReason.length() != 0) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"CONTROLLER_NOT_READY\"}");
    return;
  }

  if (motionOrSquareIsBusy()) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"BUSY\"}");
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    sendJson(503, "{\"accepted\":false,\"reason\":\"wifi\"}");
    return;
  }

  const int8_t newIndex = reserveCommandRecord();
  if (newIndex < 0) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"COMMAND_HISTORY_FULL\"}");
    return;
  }

  CommandRecord &command = commandHistory[newIndex];
  command.commandId = commandId;
  command.endpoint = endpoint;
  command.hasDistance = hasDistance;
  command.distanceM = distanceM;
  command.hasAngle = hasAngle;
  command.angleDeg = angleDeg;
  command.state = CommandState::ACCEPTED;

  requestedDirection = direction;
  requestedTargetDistanceM = hasDistance
      ? distanceM
      : TURN_EFFECTIVE_TRACK_WIDTH_M * (angleDeg * PI / 180.0f) * 0.5f;
  requestedTimeoutMs = hasDistance
      ? BOUNDED_FORWARD_TIMEOUT_MS
      : BOUNDED_TURN_TIMEOUT_MS;
  requestedTurnUsesEffectiveTrack = !hasDistance;
  requestedTurnAngleRad = hasAngle ? angleDeg * PI / 180.0f : 0.0f;
  configureBoundedCommand(
      command,
      direction,
      requestedTargetDistanceM,
      requestedTimeoutMs
  );
  activeCommandIndex = newIndex;
  startRequested = true;

  String body = "{\"accepted\":true,\"command_id\":\"";
  body += commandId;
  body += "\",\"command_state\":\"ACCEPTED\"}";
  sendJson(202, body);
}

void handleMoveForward()
{
  if (!hasExactlyArguments(
          "boot_session_id", "command_id", "distance_m")) {
    sendJson(400, "{\"accepted\":false,\"reason\":\"INVALID_ARGUMENTS\"}");
    return;
  }

  float distanceM = 0.0f;
  if (!parseFiniteFloatStrict(server.arg("distance_m"), distanceM) ||
      distanceM < BOUNDED_FORWARD_MIN_DISTANCE_M ||
      distanceM > BOUNDED_FORWARD_MAX_DISTANCE_M) {
    sendJson(400, "{\"accepted\":false,\"reason\":\"INVALID_DISTANCE_M\"}");
    return;
  }

  queueBoundedMotionRequest(
      "/move-forward",
      MotionDirection::FORWARD,
      server.arg("boot_session_id"),
      server.arg("command_id"),
      true,
      distanceM,
      false,
      0.0f
  );
}

void handleTurnRequest(MotionDirection direction, const String &endpoint)
{
  if (server.args() == 0) {
    queueLegacyMotionRequest(direction);
    return;
  }

  if (!hasExactlyArguments(
          "boot_session_id", "command_id", "angle_deg")) {
    sendJson(400, "{\"accepted\":false,\"reason\":\"INVALID_ARGUMENTS\"}");
    return;
  }

  float angleDeg = 0.0f;
  if (!parseFiniteFloatStrict(server.arg("angle_deg"), angleDeg) ||
      angleDeg < BOUNDED_TURN_MIN_ANGLE_DEG ||
      angleDeg > BOUNDED_TURN_MAX_ANGLE_DEG) {
    sendJson(400, "{\"accepted\":false,\"reason\":\"INVALID_ANGLE_DEG\"}");
    return;
  }

  queueBoundedMotionRequest(
      endpoint,
      direction,
      server.arg("boot_session_id"),
      server.arg("command_id"),
      false,
      0.0f,
      true,
      angleDeg
  );
}

void handleTurnLeft()
{
  handleTurnRequest(MotionDirection::TURN_LEFT, "/turn-left");
}

void handleTurnRight()
{
  handleTurnRequest(MotionDirection::TURN_RIGHT, "/turn-right");
}

void handleSquare()
{
  if (boundedFaultLatched) {
    sendJson(409, "{\"accepted\":false,\"reason\":\"BOUNDED_FAULT_LATCHED\"}");
    return;
  }

  if (motionIsActive() || startRequested ||
      squareIsActive() || squareRequested) {
    server.send(
        409,
        "application/json",
        "{\"accepted\":false,\"reason\":\"busy\"}"
    );
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    server.send(
        503,
        "application/json",
        "{\"accepted\":false,\"reason\":\"wifi\"}"
    );
    return;
  }

  squareRequested = true;

  server.send(
      202,
      "application/json",
      "{\"accepted\":true,\"sequence\":\"square\"}"
  );
}

void handleStop()
{
  // Cut drive immediately inside the HTTP handler. The state machine records
  // REMOTE_STOP on its next loop iteration.
  stopMotors();

  if (activeCommand() != nullptr) {
    startRequested = false;
    stopRequested = false;
    motionElapsedMs = motionStartTime == 0 ? 0 : millis() - motionStartTime;
    rightSpeedMps = 0.0f;
    leftSpeedMps = 0.0f;
    profileSpeedMps = 0.0f;
    motionState = MotionState::READY;
    completeActiveCommand(CommandState::STOPPED);
  }

  if (motionIsActive()) {
    stopRequested = true;
  }

  if (squareIsActive()) {
    failSquareSequence("REMOTE_STOP");
  }

  server.send(
      200,
      "application/json",
      "{\"stopped\":true}"
  );
}

void handleAckFault()
{
  if (!boundedFaultLatched) {
    sendJson(409, "{\"acknowledged\":false,\"reason\":\"NO_BOUNDED_FAULT\"}");
    return;
  }

  // Assert the safe electrical output first. Recovery is only allowed when
  // there is no active bounded command, no active square sequence, and the
  // controller still represents the terminal bounded fault being acknowledged.
  stopMotors();
  if (activeBoundedMotion || activeCommand() != nullptr ||
      motionOrSquareIsBusy() || rightPwm != 0 || leftPwm != 0) {
    sendJson(409, "{\"acknowledged\":false,\"reason\":\"MOTION_NOT_STOPPED\"}");
    return;
  }

  if (motionState != MotionState::FAULT || faultReason.length() == 0) {
    sendJson(409, "{\"acknowledged\":false,\"reason\":\"RECOVERY_STATE_INVALID\"}");
    return;
  }

  // Keep command-history diagnostics, the session ID, and the idempotency
  // ledger intact. Only the current fault state is acknowledged and cleared.
  const String previousReason = boundedFaultReason;
  boundedFaultLatched = false;
  boundedFaultReason = "";
  faultReason = "";
  motionState = MotionState::READY;

  String body = "{\"acknowledged\":true,\"previous_fault_reason\":\"";
  body += previousReason;
  body += "\"}";
  sendJson(200, body);
}

void handleStatus()
{
  server.send(
      200,
      "application/json",
      buildStatusJson()
  );
}

// ---------------------------------------------------------------------------
// Wi-Fi and Arduino lifecycle
// ---------------------------------------------------------------------------

void connectWiFi()
{
  stopMotors();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("sie-base");
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    stopMotors();
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WIFI_CONNECTED");
  Serial.print("IP_ADDRESS=");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("sie-base")) {
    Serial.println("MDNS_READY");
  } else {
    Serial.println("MDNS_FAILED");
  }
}

void setup()
{
  // Establish a safe motor state before starting Serial or Wi-Fi.
  pinMode(ZK_D0, OUTPUT);
  pinMode(ZK_D1, OUTPUT);
  pinMode(ZK_D2, OUTPUT);
  pinMode(ZK_D3, OUTPUT);
  stopMotors();

  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_A),
      onRightEncoderA,
      RISING
  );

  attachInterrupt(
      digitalPinToInterrupt(LEFT_A),
      onLeftEncoderA,
      RISING
  );

  Serial.begin(115200);
  delay(500);

  bootSessionId = generateBootSessionId();

  Serial.print("FIRMWARE=");
  Serial.println(FIRMWARE_ID);

  connectWiFi();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/reverse", HTTP_POST, handleReverse);
  server.on("/move-forward", HTTP_POST, handleMoveForward);
  server.on("/turn-left", HTTP_POST, handleTurnLeft);
  server.on("/turn-right", HTTP_POST, handleTurnRight);
  server.on("/square", HTTP_POST, handleSquare);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/ack-fault", HTTP_POST, handleAckFault);
  server.on("/status", HTTP_GET, handleStatus);

  server.onNotFound([]() {
    server.send(404, "application/json", "{\"accepted\":false,\"reason\":\"NOT_FOUND\"}");
  });

  server.begin();

  Serial.println("SIE_BASE_WEB_READY");
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  if (startRequested) {
    startRequested = false;

    if (!motionIsActive()) {
      const bool requestedIsTurn =
          requestedDirection == MotionDirection::TURN_LEFT ||
          requestedDirection == MotionDirection::TURN_RIGHT;
      startMotion(
          requestedDirection,
          requestedTargetDistanceM,
          requestedTimeoutMs,
          requestedTurnUsesEffectiveTrack,
          requestedIsTurn ? requestedTurnAngleRad : 0.0f
      );
    }
  }

  if (squareRequested) {
    squareRequested = false;

    if (!motionIsActive() && !squareIsActive()) {
      startSquareSequence();
    }
  }

  updateMotion();
  updateSquareSequence();
  delay(1);
}
