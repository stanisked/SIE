#include "command_replay_guard.h"

#include <cassert>
#include <cstdio>
#include <string>

int main() {
  CommandReplayGuard guard;
  assert(guard.count() == 0);
  for (size_t i = 0; i < CommandReplayGuard::kCapacity; ++i) {
    const std::string id = "approach-" + std::to_string(i);
    assert(!guard.contains(id.c_str(), id.size()));
    assert(guard.remember(id.c_str(), id.size()));
    assert(guard.contains(id.c_str(), id.size()));
    assert(!guard.remember(id.c_str(), id.size()));
  }
  assert(guard.full());
  assert(guard.count() == CommandReplayGuard::kCapacity);
  assert(guard.contains("approach-0", 10));
  assert(!guard.remember("approach-new", 12));
  assert(guard.count() == CommandReplayGuard::kCapacity);

  bool terminal[16] = {};
  for (bool &slot : terminal) slot = true;
  // Last completed command must remain available to /status. The next
  // accepted command can replace an older record, but not the active one.
  assert(selectReusableCommandSlot(16, 0, -1, 15,
      [&](uint8_t i) { return terminal[i]; }) == 0);
  assert(selectReusableCommandSlot(16, 1, 2, 1,
      [&](uint8_t i) { return terminal[i]; }) == 3);
  for (bool &slot : terminal) slot = false;
  assert(selectReusableCommandSlot(16, 0, -1, -1,
      [&](uint8_t i) { return terminal[i]; }) == -1);
  puts("command replay guard: PASS");
}
