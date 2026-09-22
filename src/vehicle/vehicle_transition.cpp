// SPDX-License-Identifier: Apache-2.0
// QuadPlane VTOL-to-fixed-wing command admission and authoritative completion.

#include "nomad/vehicle/vehicle.hpp"

#include "command_ids.hpp"

#include <chrono>
#include <optional>
#include <string>

namespace nomad::vehicle {
namespace {

constexpr auto kVtolStateFreshnessTimeout = std::chrono::seconds(3);

} // namespace

CommandResult Vehicle::transition_to_fixed_wing() {
    const auto admission = require_operation(VehicleOperation::TransitionToFixedWing);
    if (!admission.success) {
        return admission;
    }

    const auto initial_state = connection_.get_state();
    if (const auto error = vtol_transition_state_error(initial_state, initial_state.system_id, true);
        error.has_value()) {
        return {false, "transition to fixed wing rejected: " + *error};
    }

    const auto command_state_timestamp = initial_state.vtol_state_updated_at;
    const auto result = send_command(
        make_command(kQuadplaneTransitionCommand,
                     {static_cast<float>(static_cast<std::uint8_t>(telemetry::VtolState::FixedWing)), 0, 0, 0, 0, 0,
                      0}),
        "transition to fixed wing");
    if (!result.success) {
        return result;
    }
    return wait_for_fixed_wing_transition(initial_state.system_id, command_state_timestamp);
}

std::optional<std::string> Vehicle::vtol_transition_state_error(const telemetry::VehicleState &state,
                                                                std::uint8_t expected_system_id,
                                                                bool require_precondition) const {
    if (state.identity.aircraft_class != telemetry::AircraftClass::QuadPlane) {
        return "aircraft identity changed";
    }
    if (expected_system_id == 0 || state.system_id != expected_system_id) {
        return "vehicle session changed";
    }
    if (!state.connected || !state.heartbeat_fresh) {
        return "heartbeat is stale";
    }
    if (!state.vtol_state_valid || state.vtol_state_updated_at == std::chrono::steady_clock::time_point{}) {
        return "VTOL state is unavailable";
    }
    if (std::chrono::steady_clock::now() - state.vtol_state_updated_at > kVtolStateFreshnessTimeout) {
        return "VTOL state feed is stale";
    }
    if (!require_precondition) {
        return {};
    }
    if (!state.armed) {
        return "vehicle disarmed";
    }
    if (!telemetry::is_auto_mode(telemetry::AircraftClass::QuadPlane, state.custom_mode)) {
        return "AUTO mode is required";
    }
    if (state.vtol_state != telemetry::VtolState::Multicopter) {
        return "vehicle is not in multicopter VTOL state";
    }
    return {};
}

CommandResult Vehicle::wait_for_fixed_wing_transition(
    std::uint8_t expected_system_id, std::chrono::steady_clock::time_point command_state_timestamp) {
    const auto verdict = [this, expected_system_id, command_state_timestamp](const telemetry::VehicleState &state)
        -> std::optional<CommandResult> {
        if (const auto error = vtol_transition_state_error(state, expected_system_id, false); error.has_value()) {
            return CommandResult{false, "transition to fixed wing verification failed: " + *error};
        }
        if (state.vtol_state_updated_at <= command_state_timestamp) {
            return std::nullopt;
        }
        switch (state.vtol_state) {
        case telemetry::VtolState::FixedWing:
            return CommandResult{true, "transition to fixed wing verified"};
        case telemetry::VtolState::TransitionToFixedWing:
        case telemetry::VtolState::Multicopter:
            return std::nullopt;
        case telemetry::VtolState::TransitionToMulticopter:
            return CommandResult{false, "transition to fixed wing verification failed: vehicle transitioned back"};
        case telemetry::VtolState::Undefined:
            return CommandResult{false, "transition to fixed wing verification failed: VTOL state is undefined"};
        }
        return CommandResult{false, "transition to fixed wing verification failed: unknown VTOL state"};
    };
    return wait_for_state_until(
        transition_state_timeout_,
        "transition to fixed wing acknowledgement received but fixed-wing state verification timed out", verdict);
}

} // namespace nomad::vehicle
