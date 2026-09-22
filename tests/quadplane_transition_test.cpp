// SPDX-License-Identifier: Apache-2.0

#include "fake_connection.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#include <array>
#include <chrono>
#include <cstdint>

namespace {

void configure_quadplane_transition_state(FakeConnection &connection,
                                           nomad::telemetry::VtolState vtol_state =
                                               nomad::telemetry::VtolState::Multicopter) {
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                            nomad::telemetry::kVtolQuadrotor);
    connection.state->connected = true;
    connection.state->heartbeat_fresh = true;
    connection.state->armed = true;
    connection.state->custom_mode = 10; // ArduPlane AUTO.
    connection.state->vtol_state = vtol_state;
    connection.state->vtol_state_valid = true;
    connection.state->vtol_state_updated_at = std::chrono::steady_clock::now();
}

void test_quadplane_transition_constructs_command_and_verifies_fixed_wing_state() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(result.success);
    CHECK(result.message == "transition to fixed wing verified");
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.command_history.front().id == 3000);
    CHECK(connection.command_history.front().parameters[0] == 4.0F);
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::FixedWing);
}

void test_quadplane_transition_requires_auto_and_fresh_authoritative_state() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.state->custom_mode = 15;
    nomad::vehicle::Vehicle vehicle(connection);

    auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected: AUTO mode is required");
    CHECK(connection.command_history.empty());

    connection.state->custom_mode = 10;
    connection.state->vtol_state_valid = false;
    result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected: VTOL state is unavailable");
    CHECK(connection.command_history.empty());

    connection.state->vtol_state_valid = true;
    connection.state->vtol_state_updated_at = std::chrono::steady_clock::now() - std::chrono::seconds(4);
    connection.auto_stamp_fresh_fields = false;
    result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected: VTOL state feed is stale");
    CHECK(connection.command_history.empty());
}

void test_quadplane_transition_propagates_ack_failure() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.complete_transition_on_command = false;
    connection.acknowledgement = nomad::mavlink::CommandAck{3000, 4};
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing rejected by ArduPilot");
    CHECK(connection.command_history.size() == 1);
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::Multicopter);
}

void test_quadplane_transition_fails_when_state_disappears_after_ack() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.transition_state_unavailable_on_command = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: VTOL state is unavailable");
    CHECK(connection.command_history.size() == 1);
}

void test_quadplane_transition_ack_without_completion_times_out() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.complete_transition_on_command = false;
    nomad::vehicle::Vehicle vehicle(connection, {}, {}, {}, std::chrono::milliseconds(2000),
                                    std::chrono::seconds(30), std::chrono::milliseconds(20));

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message ==
          "transition to fixed wing acknowledgement received but fixed-wing state verification timed out");
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::Multicopter);
}

void test_quadplane_transition_intermediate_state_times_out() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.transition_stays_intermediate = true;
    nomad::vehicle::Vehicle vehicle(connection, {}, {}, {}, std::chrono::milliseconds(2000),
                                    std::chrono::seconds(30), std::chrono::milliseconds(20));

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message ==
          "transition to fixed wing acknowledgement received but fixed-wing state verification timed out");
    CHECK(connection.state->vtol_state == nomad::telemetry::VtolState::TransitionToFixedWing);
}

void test_quadplane_transition_fails_closed_on_state_interruption() {
    FakeConnection connection;
    connection.connect();
    configure_quadplane_transition_state(connection);
    connection.transition_loses_link_on_command = true;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.transition_to_fixed_wing();

    CHECK(!result.success);
    CHECK(result.message == "transition to fixed wing verification failed: heartbeat is stale");
    CHECK(connection.command_history.size() == 1);
}

void test_unsupported_transition_aircrafts_reject_before_transmission() {
    const std::array aircraft_types{
        nomad::telemetry::kQuadrotor,
        nomad::telemetry::kFixedWing,
        std::uint8_t{0},
    };
    for (const auto vehicle_type : aircraft_types) {
        FakeConnection connection;
        connection.connect();
        if (vehicle_type == 0) {
            connection.state->identity = {};
        } else {
            connection.state->identity =
                nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot, vehicle_type);
        }
        nomad::vehicle::Vehicle vehicle(connection);

        const auto result = vehicle.transition_to_fixed_wing();

        CHECK(!result.success);
        CHECK(connection.command_history.empty());
    }
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_quadplane_transition_constructs_command_and_verifies_fixed_wing_state();
        test_quadplane_transition_requires_auto_and_fresh_authoritative_state();
        test_quadplane_transition_propagates_ack_failure();
        test_quadplane_transition_fails_when_state_disappears_after_ack();
        test_quadplane_transition_ack_without_completion_times_out();
        test_quadplane_transition_intermediate_state_times_out();
        test_quadplane_transition_fails_closed_on_state_interruption();
        test_unsupported_transition_aircrafts_reject_before_transmission();
    });
}
