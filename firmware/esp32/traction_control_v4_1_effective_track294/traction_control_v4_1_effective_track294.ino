#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

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

constexpr char FIRMWARE_ID[] = "traction_control_v4_1_effective_track294";

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

MotionState motionState = MotionState::READY;
MotionDirection requestedDirection = MotionDirection::FORWARD;
MotionDirection activeDirection = MotionDirection::FORWARD;

float targetDistanceM = FORWARD_TARGET_DISTANCE_M;
uint32_t motionTimeoutMs = FORWARD_TIMEOUT_MS;

bool startRequested = false;
bool stopRequested = false;
bool squareRequested = false;
bool activeTurnUsesEffectiveTrack = false;

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
    case MotionState::COASTING: return "COASTING";
    case MotionState::SUCCESS:  return "SUCCESS";
    case MotionState::FAULT:    return "FAULT";
  }

  return "UNKNOWN";
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

// ---------------------------------------------------------------------------
// Motion-state transitions
// ---------------------------------------------------------------------------

void abortMotion(const String &reason)
{
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

  Serial.print("MOTION_FAULT=");
  Serial.println(reason);
}

void startMotion(MotionDirection direction)
{
  stopMotors();
  activeDirection = direction;
  activeTurnUsesEffectiveTrack =
      (direction == MotionDirection::TURN_LEFT ||
       direction == MotionDirection::TURN_RIGHT) &&
      squareIsActive();

  if (direction == MotionDirection::FORWARD) {
    targetDistanceM = FORWARD_TARGET_DISTANCE_M;
    motionTimeoutMs = FORWARD_TIMEOUT_MS;
  } else if (direction == MotionDirection::REVERSE) {
    targetDistanceM = REVERSE_TARGET_DISTANCE_M;
    motionTimeoutMs = REVERSE_TIMEOUT_MS;
  } else {
    targetDistanceM = activeTurnUsesEffectiveTrack
                          ? TURN_SQUARE_TARGET_WHEEL_DISTANCE_M
                          : TURN_SINGLE_TARGET_WHEEL_DISTANCE_M;
    motionTimeoutMs = TURN_TIMEOUT_MS;
  }

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

  Serial.println("MOTION_BRAKING");
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

  Serial.println("MOTION_SUCCESS");
  Serial.print("average_distance_m=");
  Serial.println(averageDistanceM, 4);
  Serial.print("heading_rad=");
  Serial.println(headingRad, 4);
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

  if (!rightBreakawayDetected &&
      rightCount >= BREAKAWAY_DETECT_COUNTS) {
    rightBreakawayDetected = true;
    rightBreakawayPwm = rightPwm;

    Serial.print("RIGHT_BREAKAWAY_PWM=");
    Serial.println(rightBreakawayPwm);
  }

  if (!leftBreakawayDetected &&
      leftCount >= BREAKAWAY_DETECT_COUNTS) {
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
      BREAKAWAY_MAX_PWM
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

  if (remainingDistance <= currentStopMarginM()) {
    beginCoasting();
    return;
  }

  const float hardDistanceExtra =
      isTurnCommand()
          ? TURN_HARD_DISTANCE_EXTRA_M
          : HARD_DISTANCE_EXTRA_M;

  if (averageDistanceM >
      targetDistanceM + hardDistanceExtra) {
    abortMotion("DISTANCE_LIMIT");
    return;
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

  setDrivePwm(rightPwm, leftPwm);
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

  const uint32_t now = millis();

  // The PI loop runs every 100 ms, but the terminal distance guard must react
  // faster while the calibrated traction floor is active.
  if (motionState == MotionState::DRIVING) {
    updateDistancesFromEncoders();

    if (targetDistanceM - averageDistanceM <= currentStopMarginM()) {
      beginCoasting();
      return;
    }
  }

  if (now - motionStartTime >= motionTimeoutMs) {
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
  startMotion(MotionDirection::FORWARD);
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

    startMotion(nextDirection);
  }
}

// ---------------------------------------------------------------------------
// HTTP telemetry and UI
// ---------------------------------------------------------------------------

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
  json.reserve(1200);

  json += "{";

  json += "\"firmware\":\"";
  json += FIRMWARE_ID;
  json += "\",";

  json += "\"state\":\"";
  json += stateName();
  json += "\",";

  json += "\"direction\":\"";
  json += directionName();
  json += "\",";

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
                            ? TURN_TARGET_ANGLE_RAD
                            : -TURN_TARGET_ANGLE_RAD)
                     : 0.0f, 4);
  json += ",";

  json += "\"turn_error_rad\":";
  json += String(isTurnCommand()
                     ? ((activeDirection == MotionDirection::TURN_LEFT
                             ? TURN_TARGET_ANGLE_RAD
                             : -TURN_TARGET_ANGLE_RAD) - headingRad)
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

void handleMotionRequest(MotionDirection direction)
{
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

  requestedDirection = direction;
  startRequested = true;

  server.send(
      202,
      "application/json",
      "{\"accepted\":true}"
  );
}

void handleStart()
{
  handleMotionRequest(MotionDirection::FORWARD);
}

void handleReverse()
{
  handleMotionRequest(MotionDirection::REVERSE);
}

void handleTurnLeft()
{
  handleMotionRequest(MotionDirection::TURN_LEFT);
}

void handleTurnRight()
{
  handleMotionRequest(MotionDirection::TURN_RIGHT);
}

void handleSquare()
{
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

  Serial.print("FIRMWARE=");
  Serial.println(FIRMWARE_ID);

  connectWiFi();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/reverse", HTTP_POST, handleReverse);
  server.on("/turn-left", HTTP_POST, handleTurnLeft);
  server.on("/turn-right", HTTP_POST, handleTurnRight);
  server.on("/square", HTTP_POST, handleSquare);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/status", HTTP_GET, handleStatus);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
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
      startMotion(requestedDirection);
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
