// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <chrono>
#include <cstdint>

namespace nomad::safety {

enum class ControlSource { inhibited, pilot, joystick, autonomous };

struct SelectorRange {
    int minimum{};
    int maximum{};
};

// Qualified physical observations must be supplied independently of overridden RC_CHANNELS.
// The caller serializes observations, revocations and the final send check under one lock.
class ControlSourceGate {
  public:
    using Clock = std::chrono::steady_clock;
    ControlSourceGate(std::array<SelectorRange, 3> ranges, std::chrono::milliseconds freshness);
    void observe(int selector, bool override_gate, bool physical_input_valid, Clock::time_point observed_at);
    bool admit(ControlSource source, Clock::time_point now);
    bool allows(ControlSource source, std::uint64_t generation, Clock::time_point now,
                Clock::time_point sample_at, bool deadman);
    void invalidate();
    ControlSource requested(Clock::time_point now);
    std::uint64_t generation() const;

  private:
    ControlSource parse(int value) const;
    void expire(Clock::time_point now);
    std::array<SelectorRange, 3> ranges_;
    std::chrono::milliseconds freshness_;
    Clock::time_point observed_at_{};
    ControlSource requested_{ControlSource::inhibited};
    ControlSource admitted_{ControlSource::inhibited};
    std::uint64_t generation_{1};
    bool configured_{};
};

} // namespace nomad::safety
