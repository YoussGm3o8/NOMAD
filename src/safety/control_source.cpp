// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/control_source.hpp"

namespace nomad::safety {

ControlSourceGate::ControlSourceGate(std::array<SelectorRange, 3> ranges, std::chrono::milliseconds freshness)
    : ranges_(ranges), freshness_(freshness) {
    configured_ = freshness.count() > 0;
    for (std::size_t index = 0; index < ranges.size(); ++index) {
        const auto range = ranges[index];
        if (range.minimum <= 800 || range.maximum >= 2200 || range.minimum > range.maximum) {
            configured_ = false;
        }
        if (index > 0 && ranges[index - 1].maximum >= range.minimum) {
            configured_ = false;
        }
    }
}

ControlSource ControlSourceGate::parse(int value) const {
    if (!configured_) {
        return ControlSource::inhibited;
    }
    constexpr std::array sources{ControlSource::pilot, ControlSource::joystick, ControlSource::autonomous};
    for (std::size_t index = 0; index < ranges_.size(); ++index) {
        if (value >= ranges_[index].minimum && value <= ranges_[index].maximum) {
            return sources[index];
        }
    }
    return ControlSource::inhibited;
}

void ControlSourceGate::observe(int selector, bool override_gate, bool physical_input_valid,
                               Clock::time_point observed_at) {
    auto next = physical_input_valid ? parse(selector) : ControlSource::inhibited;
    const bool software = next == ControlSource::joystick || next == ControlSource::autonomous;
    if (next != ControlSource::inhibited && override_gate != software) {
        next = ControlSource::inhibited;
    }
    if (observed_at <= observed_at_) {
        invalidate();
        return;
    }
    if (observed_at_ != Clock::time_point{} && observed_at - observed_at_ >= freshness_) {
        invalidate();
    }
    if (next != requested_) {
        invalidate();
        requested_ = next;
    }
    observed_at_ = observed_at;
}

void ControlSourceGate::invalidate() {
    ++generation_;
    requested_ = ControlSource::inhibited;
    admitted_ = ControlSource::inhibited;
}

void ControlSourceGate::expire(Clock::time_point now) {
    if (observed_at_ == Clock::time_point{} || now < observed_at_ || now - observed_at_ >= freshness_) {
        if (requested_ != ControlSource::inhibited || admitted_ != ControlSource::inhibited) {
            invalidate();
        }
    }
}

ControlSource ControlSourceGate::requested(Clock::time_point now) {
    expire(now);
    return requested_;
}

bool ControlSourceGate::admit(ControlSource source, Clock::time_point now) {
    expire(now);
    if ((source != ControlSource::joystick && source != ControlSource::autonomous) || source != requested_) {
        return false;
    }
    ++generation_;
    admitted_ = source;
    return true;
}

bool ControlSourceGate::allows(ControlSource source, std::uint64_t generation, Clock::time_point now,
                              Clock::time_point sample_at, bool deadman) {
    expire(now);
    if ((source != ControlSource::joystick && source != ControlSource::autonomous) ||
        source != requested_ || source != admitted_ || generation != generation_) {
        return false;
    }
    if (source == ControlSource::joystick && (!deadman || sample_at > now || now - sample_at >= freshness_)) {
        invalidate();
        return false;
    }
    return true;
}

std::uint64_t ControlSourceGate::generation() const {
    return generation_;
}

} // namespace nomad::safety
