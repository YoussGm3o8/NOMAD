// SPDX-License-Identifier: Apache-2.0
#include "fake_connection.hpp"
#include "nomad/mission/executor.hpp"
#include "nomad/vehicle/vehicle.hpp"
#include "test_harness.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void test_state_is_available_through_vehicle() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto state = vehicle.wait_for_state(std::chrono::seconds(1));

    CHECK(state.has_value());
    CHECK(state->connected);
    CHECK(state->system_id == 1);
    CHECK(state->identity.aircraft_class == nomad::telemetry::AircraftClass::Copter);
}

void test_state_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto state = vehicle.wait_for_state(std::chrono::seconds(1));

    CHECK(!state.has_value());
}

void test_arm_sends_arm_command() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{400, 0};

    const auto result = vehicle.arm();

    CHECK(result.success);
    CHECK(result.message == "arm verified");
    CHECK(connection.last_command.id == 400);
    CHECK(connection.last_command.parameters[0] == 1.0F);
}

void test_takeoff_rejects_invalid_altitude() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.takeoff(0.0F);

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
}

void test_mode_and_takeoff_are_verified() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{176, 0};

    const auto mode_result = vehicle.set_mode(4);
    CHECK(mode_result.success);
    CHECK(mode_result.message == "set mode verified");

    connection.acknowledgement = nomad::mavlink::CommandAck{22, 0};
    const auto takeoff_result = vehicle.takeoff(10.0F);
    CHECK(takeoff_result.success);
    CHECK(takeoff_result.message == "takeoff verified");
}

void test_land_and_rtl_are_verified() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);

    connection.acknowledgement = nomad::mavlink::CommandAck{21, 0};
    CHECK(vehicle.land().success);
    connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
    CHECK(vehicle.return_to_launch().success);
}

void test_plane_uses_plane_mode_semantics() {
    FakeConnection connection;
    connection.connect();
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                            nomad::telemetry::kFixedWing);
    nomad::vehicle::Vehicle vehicle(connection);

    connection.acknowledgement = nomad::mavlink::CommandAck{176, 0};
    CHECK(vehicle.set_guided_mode().success);
    CHECK(connection.state->custom_mode == 15);

    connection.set_mode(10); // AUTO alone must not satisfy landing verification.
    connection.acknowledgement = nomad::mavlink::CommandAck{21, 0};
    const auto command_count_before_land = connection.command_history.size();
    const auto land_result = vehicle.land();
    CHECK(!land_result.success);
    CHECK(land_result.message == "land is not qualified for Plane aircraft");
    CHECK(connection.command_history.size() == command_count_before_land);
    CHECK(connection.state->custom_mode == 10);

    connection.acknowledgement = nomad::mavlink::CommandAck{20, 0};
    CHECK(vehicle.return_to_launch().success);
    CHECK(connection.state->custom_mode == 11);
}

void test_quadplane_land_requires_qland_mode() {
    FakeConnection connection;
    connection.connect();
    connection.state->identity =
        nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                            nomad::telemetry::kVtolQuadrotor);
    nomad::vehicle::Vehicle vehicle(connection);

    connection.acknowledgement = nomad::mavlink::CommandAck{21, 0};
    CHECK(vehicle.land().success);
    CHECK(connection.state->custom_mode == 20);
}

void test_unknown_aircraft_rejects_aircraft_specific_commands() {
    FakeConnection connection;
    connection.connect();
    connection.state->identity = {};
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{176, 0};

    CHECK(!vehicle.set_guided_mode().success);
    CHECK(!vehicle.set_mode(4).success);
    CHECK(!vehicle.takeoff(5.0F).success);
    CHECK(!vehicle.goto_location({45.5, -73.6, 5.0F}).success);
    CHECK(!vehicle.land().success);
    CHECK(!vehicle.return_to_launch().success);
    CHECK(connection.command_history.empty());
    CHECK(!connection.last_goto.has_value());
}

void test_disarm_is_verified() {
    FakeConnection connection;
    connection.connect();
    connection.state->armed = true;
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{400, 0};

    const auto result = vehicle.disarm();

    CHECK(result.success);
    CHECK(!connection.state->armed);
}

void test_goto_location_validates_and_verifies() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({45.500000123, -73.600000123, 10.0F});

    CHECK(result.success);
    CHECK(connection.last_command.id == 0);
    CHECK(connection.last_goto.has_value());
    CHECK(connection.last_goto->latitude_deg == 45.500000123);
    CHECK(connection.last_goto->longitude_deg == -73.600000123);
    CHECK(connection.last_goto->relative_altitude_m == 10.0F);
}

void test_goto_location_rejects_invalid_coordinates() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 0};

    const auto result = vehicle.goto_location({91.0, 0.0, 10.0F});

    CHECK(!result.success);
    CHECK(connection.last_command.id == 0);
    CHECK(!connection.last_goto.has_value());
}

void test_goto_location_rejects_failed_action() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{192, 2};

    const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});

    CHECK(!result.success);
    CHECK(connection.last_goto.has_value());
    CHECK(connection.last_command.id == 0);
}

void test_goto_location_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.goto_location({45.5, -73.6, 10.0F});

    CHECK(!result.success);
    CHECK(!connection.last_goto.has_value());
}

void test_command_rejects_failed_acknowledgement() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    connection.acknowledgement = nomad::mavlink::CommandAck{21, 4};

    const auto result = vehicle.land();

    CHECK(!result.success);
}

void test_command_requires_connection() {
    FakeConnection connection;
    nomad::vehicle::Vehicle vehicle(connection);

    const auto result = vehicle.land();

    CHECK(!result.success);
}

void test_mission_executor_runs_steps_and_reports_progress() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    nomad::mission::MissionExecutor executor(vehicle);

    const nomad::mission::Mission mission{
        nomad::mission::Action{"guided"},
        nomad::mission::Action{"arm"},
        nomad::mission::Land{},
        nomad::mission::Action{"wait_disarmed"},
    };
    const auto result = executor.execute(mission);

    CHECK(result.success);
    CHECK(result.completed_steps == 4);
}

void test_mission_executor_rejects_unknown_action() {
    FakeConnection connection;
    connection.connect();
    nomad::vehicle::Vehicle vehicle(connection);
    nomad::mission::MissionExecutor executor(vehicle);

    const auto result = executor.execute({nomad::mission::Action{"unknown"}});

    CHECK(!result.success);
    CHECK(result.completed_steps == 0);
}

} // namespace

int main() {
    return nomad::test::run_tests([] {
        test_state_is_available_through_vehicle();
        test_state_requires_connection();
        test_arm_sends_arm_command();
        test_takeoff_rejects_invalid_altitude();
        test_mode_and_takeoff_are_verified();
        test_land_and_rtl_are_verified();
        test_plane_uses_plane_mode_semantics();
        test_quadplane_land_requires_qland_mode();
        test_unknown_aircraft_rejects_aircraft_specific_commands();
        test_disarm_is_verified();
        test_goto_location_validates_and_verifies();
        test_goto_location_rejects_invalid_coordinates();
        test_goto_location_rejects_failed_action();
        test_goto_location_requires_connection();
        test_command_rejects_failed_acknowledgement();
        test_command_requires_connection();
        test_mission_executor_runs_steps_and_reports_progress();
        test_mission_executor_rejects_unknown_action();
    });
}
