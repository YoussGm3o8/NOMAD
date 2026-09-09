// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include <mavsdk/mavsdk.hpp>
#include <plugins/telemetry/telemetry.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>

namespace {

constexpr std::string_view kDefaultEndpoint = "udpin:0.0.0.0:14550";

std::string normalize_endpoint(std::string endpoint) {
    if (endpoint.find("://") != std::string::npos) {
        return endpoint;
    }

    const auto separator = endpoint.find(':');
    if (separator == std::string::npos) {
        return endpoint;
    }

    std::string scheme = endpoint.substr(0, separator);
    if (scheme == "udp") {
        scheme = "udpin";
    }
    return scheme + "://" + endpoint.substr(separator + 1);
}

bool has_position(const mavsdk::Telemetry::Position &position) {
    return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg) &&
           std::isfinite(position.relative_altitude_m);
}

void print_usage(const char *program) {
    std::cerr << "Usage: " << program << " <connect|status> [udpin:host:port]\n";
}

std::shared_ptr<mavsdk::System> find_connected_autopilot(const mavsdk::Mavsdk &mavsdk) {
    for (const auto &system : mavsdk.systems()) {
        if (system->is_connected() && system->has_autopilot()) {
            return system;
        }
    }
    return {};
}

std::shared_ptr<mavsdk::System> connect_to_autopilot(
    mavsdk::Mavsdk &mavsdk, std::string_view endpoint) {
    const auto result = mavsdk.add_any_connection(normalize_endpoint(std::string(endpoint)));
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "MAVSDK connection failed: " << result << '\n';
        return {};
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        if (const auto system = find_connected_autopilot(mavsdk)) {
            return system;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cerr << "timed out waiting for MAVSDK autopilot heartbeat\n";
    return {};
}

int run_connect(const mavsdk::System &system) {
    std::cout << "connected system=" << static_cast<int>(system.get_system_id()) << '\n';
    return EXIT_SUCCESS;
}

int run_status(const std::shared_ptr<mavsdk::System> &system) {
    mavsdk::Telemetry telemetry(system);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    auto position = telemetry.position();
    while (!has_position(position) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        position = telemetry.position();
    }
    if (!has_position(position)) {
        std::cerr << "timed out waiting for MAVSDK position telemetry\n";
        return EXIT_FAILURE;
    }

    const auto battery = telemetry.battery();
    const auto gps = telemetry.gps_info();
    std::cout << "system=" << static_cast<int>(system->get_system_id())
              << " connected=" << (system->is_connected() ? "true" : "false")
              << " armed=" << (telemetry.armed() ? "true" : "false") << '\n';
    std::cout << "position=" << position.latitude_deg << ',' << position.longitude_deg
              << " relative_altitude_m=" << position.relative_altitude_m << '\n';
    std::cout << "battery_v=" << battery.voltage_v
              << " remaining_percent=" << battery.remaining_percent << '\n';
    std::cout << "gps_fix=" << static_cast<int>(gps.fix_type)
              << " satellites=" << gps.num_satellites << '\n';
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const auto command = std::string_view(argv[1]);
    if (command != "connect" && command != "status") {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string_view endpoint = argc == 3 ? std::string_view(argv[2]) : kDefaultEndpoint;
    mavsdk::Mavsdk mavsdk{
        mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
    const auto system = connect_to_autopilot(mavsdk, endpoint);
    if (!system) {
        return EXIT_FAILURE;
    }
    if (command == "connect") {
        return run_connect(*system);
    }
    return run_status(system);
}
