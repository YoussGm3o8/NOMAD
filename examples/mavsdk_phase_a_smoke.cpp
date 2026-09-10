// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "mavsdk_phase_a_support.hpp"

#include <mavsdk/mavsdk.hpp>
#include <plugins/telemetry/telemetry.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using nomad::mavsdk_phase_a::StatusValues;
using nomad::mavsdk_phase_a::SystemSelection;

constexpr std::string_view kDefaultEndpoint = "udpin:0.0.0.0:14550";
constexpr std::string_view kDefaultSystemId = "1";
constexpr auto kDiscoveryTimeout = std::chrono::seconds(5);
constexpr auto kTelemetryTimeout = std::chrono::seconds(8);

struct TelemetryObservation {
    std::mutex mutex;
    StatusValues values;
    std::size_t position_updates{};
    Clock::time_point first_position{};
    Clock::time_point last_position{};
};

struct ObservationSnapshot {
    StatusValues values;
    std::size_t position_updates{};
    std::int64_t observation_ms{};
    std::int64_t age_ms{};
};

struct SubscriptionHandles {
    mavsdk::Telemetry::PositionHandle position;
    mavsdk::Telemetry::BatteryHandle battery;
    mavsdk::Telemetry::GpsInfoHandle gps;
    mavsdk::Telemetry::FlightModeHandle mode;
};

void print_usage(const char *program) {
    std::cerr << "Usage: " << program << " <connect|status> [udpin:host:port] [expected-system-id]\n";
}

std::vector<std::shared_ptr<mavsdk::System>> connected_autopilots(const mavsdk::Mavsdk &mavsdk) {
    std::vector<std::shared_ptr<mavsdk::System>> systems;
    for (const auto &system : mavsdk.systems()) {
        if (system->is_connected() && system->has_autopilot()) {
            systems.push_back(system);
        }
    }
    return systems;
}

std::shared_ptr<mavsdk::System> select_expected_system(
    const std::vector<std::shared_ptr<mavsdk::System>> &systems, std::uint8_t expected_id) {
    std::vector<std::uint8_t> ids;
    for (const auto &system : systems) {
        ids.push_back(system->get_system_id());
    }
    const auto result = nomad::mavsdk_phase_a::classify_system_ids(ids, expected_id);
    if (result == SystemSelection::Selected) {
        return systems.front();
    }
    if (result == SystemSelection::WrongPeer) {
        std::cerr << "wrong autopilot peer: expected=" << static_cast<int>(expected_id)
                  << " observed=" << static_cast<int>(ids.front()) << '\n';
    } else if (result == SystemSelection::Ambiguous) {
        std::cerr << "ambiguous autopilot peers: expected=" << static_cast<int>(expected_id)
                  << " count=" << ids.size() << '\n';
    }
    return {};
}

std::shared_ptr<mavsdk::System> connect_to_autopilot(
    mavsdk::Mavsdk &mavsdk, const std::string &endpoint, std::uint8_t expected_id) {
    const auto result = mavsdk.add_any_connection(endpoint);
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "MAVSDK connection failed: " << result << '\n';
        return {};
    }
    const auto deadline = Clock::now() + kDiscoveryTimeout;
    while (Clock::now() < deadline) {
        const auto systems = connected_autopilots(mavsdk);
        if (!systems.empty()) {
            return select_expected_system(systems, expected_id);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cerr << "timed out waiting for expected MAVSDK autopilot heartbeat\n";
    return {};
}

ObservationSnapshot snapshot_observation(TelemetryObservation &observation) {
    const auto now = Clock::now();
    std::scoped_lock lock(observation.mutex);
    ObservationSnapshot snapshot{observation.values, observation.position_updates, 0, 0};
    if (observation.position_updates > 0) {
        snapshot.observation_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                observation.last_position - observation.first_position)
                .count();
        snapshot.age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - observation.last_position).count();
    }
    return snapshot;
}

void observe_position(TelemetryObservation &observation, const mavsdk::Telemetry::Position &position) {
    const auto now = Clock::now();
    std::scoped_lock lock(observation.mutex);
    observation.values.latitude_deg = position.latitude_deg;
    observation.values.longitude_deg = position.longitude_deg;
    observation.values.relative_altitude_m = position.relative_altitude_m;
    if (observation.position_updates == 0) {
        observation.first_position = now;
    }
    observation.last_position = now;
    ++observation.position_updates;
}

int run_connect(const mavsdk::System &system) {
    std::cout << "connected=true system=" << static_cast<int>(system.get_system_id()) << '\n';
    return EXIT_SUCCESS;
}

SubscriptionHandles subscribe_to_status(mavsdk::Telemetry &telemetry, TelemetryObservation &observation) {
    const auto position = telemetry.subscribe_position(
        [&observation](const auto &position) { observe_position(observation, position); });
    const auto battery = telemetry.subscribe_battery([&observation](const auto &sample) {
        std::scoped_lock lock(observation.mutex);
        observation.values.battery_voltage_v = sample.voltage_v;
        observation.values.battery_remaining_percent = sample.remaining_percent;
    });
    const auto gps = telemetry.subscribe_gps_info([&observation](const auto &sample) {
        std::scoped_lock lock(observation.mutex);
        observation.values.gps_fix_type = static_cast<int>(sample.fix_type);
        observation.values.satellites = sample.num_satellites;
    });
    const auto mode = telemetry.subscribe_flight_mode([&observation](const auto sample) {
        std::scoped_lock lock(observation.mutex);
        observation.values.flight_mode = static_cast<int>(sample);
    });
    return {position, battery, gps, mode};
}

void unsubscribe_from_status(mavsdk::Telemetry &telemetry, const SubscriptionHandles &handles) {
    telemetry.unsubscribe_position(handles.position);
    telemetry.unsubscribe_battery(handles.battery);
    telemetry.unsubscribe_gps_info(handles.gps);
    telemetry.unsubscribe_flight_mode(handles.mode);
}

bool is_qualified(const ObservationSnapshot &snapshot, const mavsdk::System &system) {
    const bool valid = nomad::mavsdk_phase_a::has_valid_status(snapshot.values);
    const bool fresh = nomad::mavsdk_phase_a::has_fresh_position_stream(
        snapshot.position_updates, snapshot.observation_ms, snapshot.age_ms);
    return valid && fresh && system.is_connected();
}

ObservationSnapshot wait_for_status(TelemetryObservation &observation, const mavsdk::System &system) {
    const auto deadline = Clock::now() + kTelemetryTimeout;
    while (Clock::now() < deadline) {
        const auto snapshot = snapshot_observation(observation);
        if (is_qualified(snapshot, system)) {
            return snapshot;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return snapshot_observation(observation);
}

void print_status(const mavsdk::System &system, const ObservationSnapshot &snapshot) {
    std::cout << "connected=true system=" << static_cast<int>(system.get_system_id())
              << " samples=" << snapshot.position_updates << " observation_ms=" << snapshot.observation_ms
              << " age_ms=" << snapshot.age_ms << '\n';
    std::cout << "position=" << snapshot.values.latitude_deg << ',' << snapshot.values.longitude_deg
              << " relative_altitude_m=" << snapshot.values.relative_altitude_m << '\n';
    std::cout << "battery_v=" << snapshot.values.battery_voltage_v
              << " remaining_percent=" << snapshot.values.battery_remaining_percent << '\n';
    std::cout << "gps_fix=" << snapshot.values.gps_fix_type << " satellites=" << snapshot.values.satellites
              << " flight_mode=" << snapshot.values.flight_mode << '\n';
}

int run_status(const std::shared_ptr<mavsdk::System> &system) {
    mavsdk::Telemetry telemetry(system);
    TelemetryObservation observation;
    const auto handles = subscribe_to_status(telemetry, observation);
    const auto final = wait_for_status(observation, *system);
    unsubscribe_from_status(telemetry, handles);
    if (is_qualified(final, *system)) {
        print_status(*system, final);
        return EXIT_SUCCESS;
    }
    std::cerr << "telemetry qualification failed: connected=" << (system->is_connected() ? "true" : "false")
              << " samples=" << final.position_updates << " observation_ms=" << final.observation_ms
              << " age_ms=" << final.age_ms << '\n';
    return EXIT_FAILURE;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argc > 4) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
    const auto command = std::string_view(argv[1]);
    if (command != "connect" && command != "status") {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const auto endpoint_input = argc >= 3 ? std::string_view(argv[2]) : kDefaultEndpoint;
    const auto endpoint = nomad::mavsdk_phase_a::canonicalize_udp_endpoint(endpoint_input);
    const auto expected_id = nomad::mavsdk_phase_a::parse_system_id(argc == 4 ? argv[3] : kDefaultSystemId);
    if (!endpoint || !expected_id) {
        std::cerr << "invalid MAVSDK endpoint or expected system ID\n";
        return EXIT_FAILURE;
    }

    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
    const auto system = connect_to_autopilot(mavsdk, *endpoint, *expected_id);
    if (!system) {
        return EXIT_FAILURE;
    }
    return command == "connect" ? run_connect(*system) : run_status(system);
}
