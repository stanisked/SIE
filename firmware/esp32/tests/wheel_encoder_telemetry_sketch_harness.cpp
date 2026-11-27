// Compiled by test_wheel_encoder_telemetry_v1.py with the selected sketch
// functions. GPIO, timing, Arduino String and UART are host simulations.
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "wheel_encoder_telemetry.h"

class String : public std::string {
 public:
  using std::string::string;
  String(const std::string &text) : std::string(text) {}
  explicit String(int code) : std::string(std::to_string(code)) {}
  int lastIndexOf(char c) const { return static_cast<int>(rfind(c)); }
  String substring(size_t begin, size_t end = npos) const
  {
    return substr(begin, end == npos ? npos : end - begin);
  }
};

constexpr auto RIGHT_B = 19;
constexpr auto LEFT_B = 22;
volatile int32_t rightRawCount = 0;
volatile int32_t leftRawCount = 0;
WheelEncoderTelemetry wheelEncoderTelemetry;
int64_t clockReads = 0;
int64_t clockStep = 1;
bool interruptEveryClockRead = false;
thread_local bool rightBHigh = false;
thread_local bool leftBHigh = false;

void onRightEncoderA();

int64_t esp_timer_get_time()
{
  if (interruptEveryClockRead) onRightEncoderA();
  clockReads += clockStep;
  return clockReads;
}

#define IRAM_ATTR
int digitalRead(int pin) { return pin == RIGHT_B ? rightBHigh : leftBHigh; }
void noInterrupts() {}
void interrupts() {}

const String bootSessionId = "0123456789ABCDEF";
String uartRequestId;
bool uartRequestActive = false;
const char *controllerState = "READY";
bool boundedFaultLatched = false;
std::atomic<bool> steeredSafetyLatched{false};
std::string reply;

const char *stateName() { return controllerState; }
String buildUartStatus() { return "{\"legacy\":\"status\"}"; }
String buildUartStatusCompact() { return "{\"legacy\":\"compact\"}"; }
bool parseFiniteFloatStrict(const String &, float &) { std::abort(); }
constexpr float BOUNDED_FORWARD_MIN_DISTANCE_M = 0.0f;
constexpr float STEERED_FORWARD_MAX_DISTANCE_M = 0.0f;
enum class MotionDirection { FORWARD };
void queueBoundedMotionRequest(...) { std::abort(); }
void applySteerUpdateValues(...) { std::abort(); }
void handleShadowStart(...) { std::abort(); }
void handleShadowPause(...) { std::abort(); }
void handleShadowResume(...) { std::abort(); }
void handleShadowTurn(...) { std::abort(); }
void handleShadowStop(...) { std::abort(); }

struct FakeUart {
  void print(const String &text)
  {
    reply += text;
  }
  void print(char c) { print(String(1, c)); }
  void println(const String &text)
  {
    print(text);
    print("\r\n");
  }
} Serial2;

void sendUartReply(int status, const String &body);
void sendJson(int status, const String &body)
{
  assert(uartRequestActive);
  sendUartReply(status, body);
}
void handleFaultDiagnostics() { sendJson(200, "{\"legacy\":\"fault\"}"); }
void handleStop() { sendJson(202, "{\"legacy\":\"stop\"}"); }

#include "wheel_encoder_sketch.inc"

String framedRequest(const String &core)
{
  return core + "|" + uartHex16(uartCrc16(core));
}

WheelEncoderSample readSample()
{
  WheelEncoderSample sample{};
  assert(sampleWheelEncoderTelemetry(sample));
  return sample;
}

void checkWheelMotion()
{
  auto sample = readSample();
  assert(sample.sequence == 1 && sample.right.ticks == 0 && sample.left.ticks == 0);

  for (const auto *state : {"READY", "STARTING", "DRIVING", "PAUSED", "BRAKING",
                            "COASTING", "FAULT", "SUCCESS"}) {
    controllerState = state;
    const auto before = readSample();
    rightBHigh = false;
    leftBHigh = true;
    onRightEncoderA();
    onLeftEncoderA();
    sample = readSample();
    assert(sample.right.ticks == before.right.ticks + 1);
    assert(sample.left.ticks == before.left.ticks + 1);
    assert(rightRawCount == -1 && leftRawCount == 1);

    rightBHigh = true;
    leftBHigh = false;
    onRightEncoderA();
    onLeftEncoderA();
    sample = readSample();
    assert(sample.right.ticks == before.right.ticks);
    assert(sample.left.ticks == before.left.ticks);
    assert(rightRawCount == 0 && leftRawCount == 0);

    rightBHigh = false;
    onRightEncoderA();
    onLeftEncoderA();
    sample = readSample();
    assert(sample.right.ticks == before.right.ticks + 1);
    assert(sample.left.ticks == before.left.ticks - 1);
    resetEncoderCounts();
    auto afterReset = readSample();
    assert(rightRawCount == 0 && leftRawCount == 0);
    assert(afterReset.right.ticks == sample.right.ticks);
    assert(afterReset.left.ticks == sample.left.ticks);

    rightBHigh = true;
    leftBHigh = true;
    onRightEncoderA();
    onLeftEncoderA();
    sample = readSample();
    assert(sample.right.ticks == afterReset.right.ticks - 1);
    assert(sample.left.ticks == afterReset.left.ticks + 1);
    resetEncoderCounts();
  }
}

void checkConcurrentSnapshots()
{
  const auto baseline = readSample();
  std::thread writer([] {
    rightBHigh = false;
    leftBHigh = true;
    for (int i = 0; i < 20'000; ++i) {
      onRightEncoderA();
      onLeftEncoderA();
    }
  });
  auto previous = baseline;
  for (int i = 0; i < 20'000; ++i) {
    WheelEncoderSample sample{};
    if (!sampleWheelEncoderTelemetry(sample)) continue;
    assert(sample.right.ticks == sample.left.ticks ||
           sample.right.ticks == sample.left.ticks + 1);
    assert(sample.right.ticks >= previous.right.ticks &&
           sample.left.ticks >= previous.left.ticks);
    assert(sample.monotonicUs > previous.monotonicUs);
    assert(sample.sequence == previous.sequence + 1);
    previous = sample;
  }
  writer.join();
  const auto final = readSample();
  assert(final.right.ticks == baseline.right.ticks + 20'000);
  assert(final.left.ticks == baseline.left.ticks + 20'000);
  assert(final.sequence == previous.sequence + 1);
}

void checkContendedSnapshot()
{
  const auto baseline = readSample();
  // Force a publication between the counter reads and sampled time on
  // every attempt. All updates survive, but no sequence is issued.
  interruptEveryClockRead = true;
  handleUartFrame(framedRequest("@1|00000020|WHEEL_ENCODERS_V1"));
  assert(reply.find("|503|") != std::string::npos);
  assert(reply.find("WHEEL_ENCODER_SNAPSHOT_UNAVAILABLE") != std::string::npos);
  interruptEveryClockRead = false;
  const auto next = readSample();
  assert(next.sequence == baseline.sequence + 1);
  assert(next.right.ticks == baseline.right.ticks + 12);
  assert(next.left.ticks == baseline.left.ticks);

  WheelEncoderSample refused{};
  clockStep = 1'000;
  assert(!sampleWheelEncoderTelemetry(refused));
  clockStep = 1;
  assert(readSample().sequence == next.sequence + 1);
}

void checkBoundaries()
{
  WheelEncoderTotal high;
  high.ticks = INT64_MAX - 1;
  high.record(true);
  assert(high.ticks == INT64_MAX && !high.overflow);
  high.record(true);
  high.record(false);
  assert(high.ticks == INT64_MAX && high.overflow);

  WheelEncoderTotal low;
  low.ticks = INT64_MIN + 1;
  low.record(false);
  assert(low.ticks == INT64_MIN && !low.overflow);
  low.record(false);
  low.record(true);
  assert(low.ticks == INT64_MIN && low.overflow);

  uint64_t sequence = UINT64_MAX - 1;
  assert(advanceWheelEncoderSequence(sequence) && sequence == UINT64_MAX);
  assert(!advanceWheelEncoderSequence(sequence) && sequence == UINT64_MAX);

  char body[WHEEL_ENCODER_JSON_CAPACITY];
  WheelEncoderTotal minValid;
  minValid.ticks = INT64_MIN;
  const WheelEncoderSample longest{UINT64_MAX, INT64_MAX, minValid, minValid, false};
  assert(formatWheelEncoderSample(
      body, longest, bootSessionId.c_str(), "CORRECTING", false, false) == 407);

  const WheelEncoderSample extremes{UINT64_MAX, INT64_MAX, low, high, true};
  const int length = formatWheelEncoderSample(
      body, extremes, bootSessionId.c_str(), "CORRECTING", false, false);
  assert(length > 0 && static_cast<size_t>(length) < sizeof(body));
  assert(length == static_cast<int>(strlen(body)));
  uartRequestId = "00000021";
  sendUartReply(200, String(body));
}

void checkDispatch()
{
  const auto before = readSample();
  handleUartFrame("@1|00000001|WHEEL_ENCODERS_V1|0000");
  handleUartFrame(framedRequest("@1|bad|WHEEL_ENCODERS_V1"));
  assert(reply.empty() && !uartRequestActive);
  handleUartFrame(framedRequest("@1|00000001|WHEEL_ENCODERS_V1|extra"));
  assert(reply.find("INVALID_UART_OPERATION") != std::string::npos);
  assert(readSample().sequence == before.sequence + 1);

  for (const auto *op : {"STATUS", "STATUS_COMPACT", "FAULT_DIAGNOSTICS", "STOP"}) {
    reply.clear();
    handleUartFrame(framedRequest(String("@1|00000001|") + op));
    assert(reply.find("\"legacy\"") != std::string::npos);
    assert(!uartRequestActive);
  }
  assert(readSample().sequence == before.sequence + 2);
  reply.clear();
  controllerState = "FAULT";
  boundedFaultLatched = true;
  steeredSafetyLatched = true;
  handleUartFrame(framedRequest("@1|00000020|WHEEL_ENCODERS_V1"));
  assert(!uartRequestActive);
  assert(readSample().sequence == before.sequence + 4);
}

int main(int argc, char **argv)
{
  assert(argc == 2);
  const std::string mode = argv[1];
  if (mode == "motion") checkWheelMotion();
  else if (mode == "concurrent") checkConcurrentSnapshots();
  else if (mode == "contended") checkContendedSnapshot();
  else if (mode == "boundaries") checkBoundaries();
  else if (mode == "dispatch") checkDispatch();
  else std::abort();
  std::cout << reply;
}
