// SPDX-License-Identifier: Apache-2.0
// The telemetry readout and the shared result printer.
//
// The verb surface, its usage text and the API-key boundary live in
// cli_command_table.hpp; argument parsing in cli_arguments.cpp; the velocity
// and demo verbs in cli_velocity.cpp and cli_demos.cpp.
#include "cli_commands.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

void print_state(const nomad::telemetry::VehicleState &state) {
    std::cout << "system=" << static_cast<int>(state.system_id) << " component=" << static_cast<int>(state.component_id)
              << " connected=" << (state.connected ? "true" : "false") << " armed=" << (state.armed ? "true" : "false")
              << " mode=" << state.custom_mode << '\n';
    if (state.position_valid) {
        std::cout << "position=" << state.position.latitude_deg << ',' << state.position.longitude_deg
                  << " altitude_m=" << state.position.altitude_m
                  << " relative_altitude_m=" << state.position.relative_altitude_m << '\n';
    }
    if (state.battery_valid) {
        std::cout << "battery_v=" << state.battery.voltage_v << " remaining_percent=" << state.battery.remaining_percent
                  << '\n';
    }
    if (state.gps_valid) {
        std::cout << "gps_fix=" << static_cast<int>(state.gps.fix_type)
                  << " satellites=" << static_cast<int>(state.gps.satellites) << '\n';
    }
}

} // namespace

int print_result(const nomad::vehicle::CommandResult &result) {
    std::cout << result.message << '\n';
    return result.success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int run_status(nomad::mavlink::MavlinkConnection &connection) {
    // Wait for the first heartbeat so the target system is known. The MAVProxy
    // link already forwards ArduPilot's default streams, so no explicit stream
    // requests are needed; requesting them only loads the link ahead of the
    // next command's acknowledgement.
    if (!connection.wait_for_heartbeat(std::chrono::seconds(6)).has_value()) {
        std::cerr << "timed out waiting for ArduPilot heartbeat\n";
        return EXIT_FAILURE;
    }

    // Give the lower-rate GPS/position messages time to arrive, then print the
    // freshest sample even when the complete set never did.
    nomad::vehicle::Vehicle vehicle(connection);
    const auto state = vehicle.wait_for_telemetry(
        std::chrono::seconds(5), [](const nomad::telemetry::VehicleState &sample) {
            return sample.position_valid && sample.gps_valid;
        });
    if (!state.has_value()) {
        std::cerr << "timed out waiting for ArduPilot telemetry\n";
        return EXIT_FAILURE;
    }
    print_state(*state);
    return EXIT_SUCCESS;
}
