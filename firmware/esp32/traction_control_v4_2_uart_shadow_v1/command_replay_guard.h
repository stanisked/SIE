#pragma once

#include <stddef.h>
#include <stdint.h>

// Every accepted command ID remains represented until the next boot. A hash
// collision only rejects a new ID; it cannot allow an old ID to run twice.
// The boot-session check prevents replay across restarts.
class CommandReplayGuard {
 public:
  static constexpr size_t kCapacity = 4096;

  bool contains(const char* id, size_t length) const {
    const uint64_t fingerprint = hash(id, length);
    for (size_t i = 0; i < count_; ++i) {
      if (fingerprints_[i] == fingerprint) return true;
    }
    return false;
  }

  bool full() const { return count_ == kCapacity; }
  size_t count() const { return count_; }

  // Call only for a new, valid ID after an execution slot is available.
  bool remember(const char* id, size_t length) {
    if (full() || contains(id, length)) return false;
    fingerprints_[count_++] = hash(id, length);
    return true;
  }

 private:
  static uint64_t hash(const char* id, size_t length) {
    uint64_t value = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < length; ++i) {
      value ^= static_cast<uint8_t>(id[i]);
      value *= UINT64_C(1099511628211);
    }
    return value;
  }

  uint64_t fingerprints_[kCapacity] = {};
  size_t count_ = 0;
};

template <typename IsTerminal>
int8_t selectReusableCommandSlot(uint8_t capacity, uint8_t cursor,
                                 int8_t active, int8_t last,
                                 IsTerminal isTerminal) {
  for (uint8_t offset = 0; offset < capacity; ++offset) {
    const uint8_t index = (cursor + offset) % capacity;
    if (index != active && index != last && isTerminal(index)) {
      return static_cast<int8_t>(index);
    }
  }
  return -1;
}

