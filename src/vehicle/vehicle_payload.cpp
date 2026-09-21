// SPDX-License-Identifier: Apache-2.0
// Payload arming and release: the consuming interlock decision plus the relay
// pulse that performs the release.
//
// Split from vehicle.cpp. The interlock itself lives in nomad/safety/payload.hpp
// so it stays testable without a transport; this file only binds it to time and
// to the relay command.
#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"

#include <chrono>
#include <thread>

namespace nomad::vehicle {
namespace {

float get_monotonic_seconds() {
    static const auto start = std::chrono::steady_clock::now();
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
}

} // namespace

CommandResult Vehicle::arm_payload() {
    std::lock_guard lock(payload_mutex_);
    const auto result = payload_interlock_.arm(get_monotonic_seconds());
    if (!result.allowed) {
        return {false, result.message};
    }
    return {true, "payload release armed"};
}

CommandResult Vehicle::release_payload(int relay_number, float duration_seconds) {
    const auto duration = safety::clamp_release_duration(duration_seconds);
    if (!duration.has_value()) {
        return {false, "release duration must be finite"};
    }
    // Validate before consuming the interlock, so a typo cannot burn an arming.
    if (!relay_number_is_valid(relay_number)) {
        return {false, kRelayRangeMessage};
    }
    const auto admission = require_operation(VehicleOperation::ReleasePayload);
    if (!admission.success) {
        return admission;
    }

    std::lock_guard lock(payload_mutex_);
    const auto decision = payload_interlock_.evaluate_release(get_monotonic_seconds());
    if (!decision.allowed) {
        return {false, decision.message};
    }
    // set_relay owns the relay command layout, so the pulse cannot drift from
    // the standalone relay verb.
    if (!set_relay(relay_number, true).success) {
        static_cast<void>(set_relay(relay_number, false));
        return {false, "payload relay on command failed"};
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(*duration));
    if (!set_relay(relay_number, false).success) {
        return {false, "payload relay off command failed"};
    }
    return {true, "payload release verified"};
}

} // namespace nomad::vehicle
