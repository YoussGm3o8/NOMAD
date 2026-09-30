// SPDX-License-Identifier: Apache-2.0
#include "nomad/safety/control_source.hpp"
#include "support/test_harness.hpp"

using nomad::safety::ControlSource;
using nomad::safety::ControlSourceGate;
using namespace std::chrono_literals;

namespace {

ControlSourceGate make_gate() {
    // Synthetic ranges test policy; these are not an aircraft channel prescription.
    return ControlSourceGate({{{980, 1020}, {1480, 1520}, {1980, 2020}}}, 200ms);
}

void test_transitions_require_admission() {
    auto gate = make_gate();
    const auto now = ControlSourceGate::Clock::time_point{} + 1s;
    CHECK(gate.requested(now) == ControlSource::inhibited);
    gate.observe(1000, false, true, now);
    CHECK(gate.requested(now) == ControlSource::pilot);
    CHECK(!gate.admit(ControlSource::joystick, now));
    gate.observe(1500, true, true, now + 1ms);
    CHECK(!gate.allows(ControlSource::joystick, gate.generation(), now + 1ms, now, true));
    CHECK(gate.admit(ControlSource::joystick, now + 1ms));
    const auto old_generation = gate.generation();
    CHECK(gate.allows(ControlSource::joystick, old_generation, now + 2ms, now, true));
    gate.observe(2000, true, true, now + 3ms);
    CHECK(!gate.allows(ControlSource::joystick, old_generation, now + 3ms, now, true));
    CHECK(!gate.allows(ControlSource::autonomous, gate.generation(), now + 3ms, now, true));
    CHECK(gate.admit(ControlSource::autonomous, now + 3ms));
    gate.observe(1000, false, true, now + 4ms);
    CHECK(!gate.allows(ControlSource::autonomous, gate.generation(), now + 4ms, now, true));
}

void test_invalid_and_stale_input() {
    const auto now = ControlSourceGate::Clock::time_point{} + 1s;
    for (const auto value : {0, 979, 1021, 1300, 1479, 1521, 1800, 1979, 2021, 65535}) {
        auto gate = make_gate();
        gate.observe(value, true, true, now);
        CHECK(gate.requested(now) == ControlSource::inhibited);
    }
    auto gate = make_gate();
    gate.observe(1500, false, true, now);
    CHECK(gate.requested(now) == ControlSource::inhibited);
    gate.observe(1500, true, false, now + 1ms);
    CHECK(!gate.admit(ControlSource::joystick, now + 1ms));
    gate.observe(1500, true, true, now + 2ms);
    CHECK(gate.admit(ControlSource::joystick, now + 2ms));
    const auto old = gate.generation();
    CHECK(!gate.allows(ControlSource::joystick, old, now + 202ms, now + 202ms, true));
    gate.observe(1500, true, true, now + 203ms);
    CHECK(!gate.allows(ControlSource::joystick, old, now + 203ms, now + 203ms, true));
}

void test_loss_does_not_restore_owner() {
    auto gate = make_gate();
    const auto now = ControlSourceGate::Clock::time_point{} + 1s;
    gate.observe(1500, true, true, now);
    CHECK(gate.admit(ControlSource::joystick, now));
    const auto old = gate.generation();
    CHECK(!gate.allows(ControlSource::joystick, old, now + 1ms, now, false));
    gate.observe(1500, true, true, now + 2ms);
    CHECK(!gate.allows(ControlSource::joystick, old, now + 2ms, now, true));
    CHECK(gate.admit(ControlSource::joystick, now + 2ms));
    gate.invalidate(); // Runtime/router/link/device loss uses the same revocation transition.
    gate.observe(1500, true, true, now + 3ms);
    CHECK(!gate.allows(ControlSource::joystick, gate.generation(), now + 3ms, now, true));
    auto restarted = make_gate();
    CHECK(!restarted.admit(ControlSource::joystick, now));
}

void test_gap_reversal_and_bad_calibration() {
    auto gate = make_gate();
    const auto now = ControlSourceGate::Clock::time_point{} + 1s;
    gate.observe(2000, true, true, now);
    CHECK(gate.admit(ControlSource::autonomous, now));
    const auto old = gate.generation();
    gate.observe(2000, true, true, now + 200ms);
    CHECK(!gate.allows(ControlSource::autonomous, old, now + 200ms, now, true));
    gate.observe(2000, true, true, now + 199ms);
    CHECK(gate.requested(now + 200ms) == ControlSource::inhibited);
    ControlSourceGate invalid({{{980, 1500}, {1500, 1520}, {1980, 2020}}}, 200ms);
    invalid.observe(1500, true, true, now);
    CHECK(!invalid.admit(ControlSource::joystick, now));
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_transitions_require_admission();
        test_invalid_and_stale_input();
        test_loss_does_not_restore_owner();
        test_gap_reversal_and_bad_calibration();
    });
}
