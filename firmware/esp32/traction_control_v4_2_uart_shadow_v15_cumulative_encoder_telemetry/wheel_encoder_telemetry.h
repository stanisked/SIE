#pragma once

#include <atomic>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>

struct WheelEncoderTotal {
  int64_t ticks = 0;
  bool overflow = false;

  // Keep checked arithmetic in the ISR with no flash-resident helper.
  __attribute__((always_inline)) inline void record(bool forward)
  {
    if (overflow) return;
    if (forward ? ticks == INT64_MAX : ticks == INT64_MIN) {
      overflow = true;
      return;
    }
    ticks += forward ? 1 : -1;
  }
};

struct WheelEncoderSample {
  uint64_t sequence;
  int64_t monotonicUs;
  WheelEncoderTotal right;
  WheelEncoderTotal left;
  bool sequenceExhausted;
};

inline bool advanceWheelEncoderSequence(uint64_t &sequence)
{
  if (sequence == UINT64_MAX) return false;
  ++sequence;
  return true;
}

class WheelEncoderTelemetry {
 public:
  __attribute__((always_inline)) inline void recordRightEdge(bool phaseBHigh)
  {
    publish(right_, !phaseBHigh);
  }

  __attribute__((always_inline)) inline void recordLeftEdge(bool phaseBHigh)
  {
    publish(left_, phaseBHigh);
  }

  // The UART loop is the only reader and sequence owner. A short window also
  // prevents a revision ABA: 2^31 GPIO callbacks cannot run within 1 ms.
  bool snapshot(int64_t (*clockUs)(), WheelEncoderSample &sample)
  {
    for (unsigned attempt = 0; attempt < 4; ++attempt) {
      const int64_t started = clockUs();
      const uint32_t before = revision_.load();
      if (before & 1U) continue;
      const WheelEncoderTotal right = right_.read();
      const WheelEncoderTotal left = left_.read();
      const int64_t sampled = clockUs();
      const uint32_t after = revision_.load();
      const int64_t finished = clockUs();
      if (before != after || started < 0 || sampled < started ||
          finished < sampled || finished - started > 1000) continue;

      const bool exhausted = !advanceWheelEncoderSequence(sequence_);
      sample = {sequence_, sampled, right, left, exhausted};
      return true;
    }
    return false;
  }

 private:
  struct PublishedWheel {
    // Only the serialized GPIO ISR callbacks access the 64-bit total.
    WheelEncoderTotal total;
    std::atomic<uint32_t> low{0};
    std::atomic<uint32_t> high{0};
    std::atomic<uint32_t> overflow{0};

    WheelEncoderTotal read() const
    {
      const uint64_t bottom = low.load();
      const uint64_t bits = (static_cast<uint64_t>(high.load()) << 32) | bottom;
      const int64_t ticks = bits <= INT64_MAX
          ? static_cast<int64_t>(bits)
          : INT64_MIN + static_cast<int64_t>(bits - (uint64_t{1} << 63));
      return {ticks, overflow.load() != 0};
    }
  };

  __attribute__((always_inline)) inline void publish(PublishedWheel &wheel, bool forward)
  {
    // Sequentially consistent 32-bit loads/stores keep the odd/even
    // publication bracket ordered on either CPU. No ISR waits for readers.
    revision_.store(++writerRevision_);
    wheel.total.record(forward);
    const uint64_t bits = static_cast<uint64_t>(wheel.total.ticks);
    wheel.low.store(static_cast<uint32_t>(bits));
    wheel.high.store(static_cast<uint32_t>(bits >> 32));
    wheel.overflow.store(wheel.total.overflow);
    revision_.store(++writerRevision_);
  }

  PublishedWheel right_;
  PublishedWheel left_;
  uint32_t writerRevision_ = 0;
  std::atomic<uint32_t> revision_{0};
  uint64_t sequence_ = 0;
};

constexpr size_t WHEEL_ENCODER_JSON_CAPACITY = 416;

inline int formatWheelEncoderSample(
    char (&body)[WHEEL_ENCODER_JSON_CAPACITY], const WheelEncoderSample &sample,
    const char *bootSessionId, const char *controllerState, bool faultLatched,
    bool steeredSafetyLatched)
{
  // Decimal strings preserve all bits for consumers with float-backed JSON.
  return snprintf(
      body, sizeof(body),
      "{\"version\":1,\"boot_session_id\":\"%.16s\","
      "\"sample_sequence\":\"%" PRIu64 "\",\"sample_monotonic_us\":\"%" PRId64 "\","
      "\"right_total_signed_ticks\":\"%" PRId64 "\","
      "\"left_total_signed_ticks\":\"%" PRId64 "\","
      "\"controller_state\":\"%.10s\",\"fault_latched\":%s,\"steered_safety_latched\":%s,"
      "\"right_counter_overflow\":%s,\"left_counter_overflow\":%s,"
      "\"sequence_exhausted\":%s}",
      bootSessionId, sample.sequence, sample.monotonicUs, sample.right.ticks,
      sample.left.ticks, controllerState, faultLatched ? "true" : "false",
      steeredSafetyLatched ? "true" : "false", sample.right.overflow ? "true" : "false",
      sample.left.overflow ? "true" : "false", sample.sequenceExhausted ? "true" : "false");
}
