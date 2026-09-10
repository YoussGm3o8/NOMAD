// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "nomad/mavlink/mavsdk_connection.hpp"

#include <mavsdk/mavsdk.hpp>
#include <plugins/telemetry/telemetry.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace nomad::mavlink {
namespace {

using Clock = std::chrono::steady_clock;

struct TelemetryObservation {
    mutable std::mutex mutex;
    mavsdk_phase_a::StatusValues values;
    std::size_t position_updates{};
    Clock::time_point first_position{};
    Clock::time_point last_position{};
};

struct SubscriptionHandles {
    mavsdk::Telemetry::PositionHandle position;
    mavsdk::Telemetry::BatteryHandle battery;
    mavsdk::Telemetry::GpsInfoHandle gps;
    mavsdk::Telemetry::FlightModeHandle mode;
};

std::vector<std::shared_ptr<mavsdk::System>> connected_autopilots(const mavsdk::Mavsdk &sdk) {
    std::vector<std::shared_ptr<mavsdk::System>> systems;
    for (const auto &system : sdk.systems()) {
        if (system->is_connected() && system->has_autopilot()) {
            systems.push_back(system);
        }
    }
    return systems;
}

MavsdkConnectionError selection_error(mavsdk_phase_a::SystemSelection selection) {
    if (selection == mavsdk_phase_a::SystemSelection::WrongPeer) {
        return MavsdkConnectionError::WrongPeer;
    }
    if (selection == mavsdk_phase_a::SystemSelection::Ambiguous) {
        return MavsdkConnectionError::AmbiguousPeers;
    }
    return MavsdkConnectionError::DiscoveryTimeout;
}

} // namespace

struct MavsdkConnection::Implementation {
    explicit Implementation(MavsdkConnectionOptions input)
        : options(std::move(input)), sdk(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {}

    MavsdkConnectionOptions options;
    mavsdk::Mavsdk sdk;
    std::optional<mavsdk::Mavsdk::ConnectionHandle> handle;
    std::shared_ptr<mavsdk::System> system;
    std::unique_ptr<mavsdk::Telemetry> telemetry;
    std::optional<SubscriptionHandles> subscriptions;
    TelemetryObservation observation;
    MavsdkConnectionError error{MavsdkConnectionError::None};

    void observe_position(const mavsdk::Telemetry::Position &position) {
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

    void subscribe() {
        telemetry = std::make_unique<mavsdk::Telemetry>(system);
        const auto position = telemetry->subscribe_position([this](const auto &value) { observe_position(value); });
        const auto battery = telemetry->subscribe_battery([this](const auto &value) {
            std::scoped_lock lock(observation.mutex);
            observation.values.battery_voltage_v = value.voltage_v;
            observation.values.battery_remaining_percent = value.remaining_percent;
        });
        const auto gps = telemetry->subscribe_gps_info([this](const auto &value) {
            std::scoped_lock lock(observation.mutex);
            observation.values.gps_fix_type = static_cast<int>(value.fix_type);
            observation.values.satellites = value.num_satellites;
        });
        const auto mode = telemetry->subscribe_flight_mode([this](const auto value) {
            std::scoped_lock lock(observation.mutex);
            observation.values.flight_mode = static_cast<int>(value);
        });
        subscriptions = SubscriptionHandles{position, battery, gps, mode};
    }

    void unsubscribe() {
        if (!telemetry || !subscriptions) {
            return;
        }
        telemetry->unsubscribe_position(subscriptions->position);
        telemetry->unsubscribe_battery(subscriptions->battery);
        telemetry->unsubscribe_gps_info(subscriptions->gps);
        telemetry->unsubscribe_flight_mode(subscriptions->mode);
        subscriptions.reset();
    }

    void close() {
        unsubscribe();
        telemetry.reset();
        system.reset();
        if (handle) {
            sdk.remove_connection(*handle);
            handle.reset();
        }
        std::scoped_lock lock(observation.mutex);
        observation.values = {};
        observation.position_updates = 0;
        observation.first_position = {};
        observation.last_position = {};
    }

    bool select_system() {
        const auto systems = connected_autopilots(sdk);
        std::vector<std::uint8_t> ids;
        for (const auto &candidate : systems) {
            ids.push_back(candidate->get_system_id());
        }
        const auto selection = mavsdk_phase_a::classify_system_ids(ids, options.expected_system_id);
        if (selection != mavsdk_phase_a::SystemSelection::Selected) {
            if (selection != mavsdk_phase_a::SystemSelection::NoAutopilot) {
                error = selection_error(selection);
            }
            return false;
        }
        system = systems.front();
        subscribe();
        return true;
    }
};

MavsdkConnection::MavsdkConnection(MavsdkConnectionOptions options)
    : implementation_(std::make_unique<Implementation>(std::move(options))) {}

MavsdkConnection::~MavsdkConnection() {
    disconnect();
}

bool MavsdkConnection::connect() {
    if (is_connected()) {
        return true;
    }
    implementation_->close();
    implementation_->error = MavsdkConnectionError::None;
    const auto endpoint = mavsdk_phase_a::canonicalize_udp_endpoint(implementation_->options.endpoint);
    if (!endpoint || implementation_->options.expected_system_id == 0 ||
        implementation_->options.discovery_timeout <= std::chrono::milliseconds::zero()) {
        implementation_->error = MavsdkConnectionError::InvalidConfiguration;
        return false;
    }
    auto [result, handle] = implementation_->sdk.add_any_connection_with_handle(*endpoint);
    if (result != mavsdk::ConnectionResult::Success) {
        implementation_->error = MavsdkConnectionError::AddConnectionFailed;
        return false;
    }
    implementation_->handle = handle;
    const auto deadline = Clock::now() + implementation_->options.discovery_timeout;
    while (Clock::now() < deadline && implementation_->error == MavsdkConnectionError::None) {
        if (implementation_->select_system()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (implementation_->error == MavsdkConnectionError::None) {
        implementation_->error = MavsdkConnectionError::DiscoveryTimeout;
    }
    implementation_->close();
    return false;
}

void MavsdkConnection::disconnect() {
    implementation_->close();
}

bool MavsdkConnection::is_connected() const {
    return implementation_->system && implementation_->system->is_connected();
}

std::uint8_t MavsdkConnection::system_id() const {
    return implementation_->system ? implementation_->system->get_system_id() : 0;
}

MavsdkConnectionError MavsdkConnection::last_error() const {
    return implementation_->error;
}

MavsdkStatusSnapshot MavsdkConnection::get_status() const {
    const auto now = Clock::now();
    std::scoped_lock lock(implementation_->observation.mutex);
    const auto &observation = implementation_->observation;
    MavsdkStatusSnapshot snapshot{observation.values, observation.position_updates, 0, 0};
    if (observation.position_updates > 0) {
        snapshot.observation_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(observation.last_position - observation.first_position)
                .count();
        snapshot.age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - observation.last_position).count();
    }
    return snapshot;
}

std::optional<MavsdkStatusSnapshot> MavsdkConnection::wait_for_status(std::chrono::milliseconds timeout) const {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline && is_connected()) {
        const auto snapshot = get_status();
        if (mavsdk_phase_a::has_valid_status(snapshot.values) &&
            mavsdk_phase_a::has_fresh_position_stream(
                snapshot.position_updates, snapshot.observation_ms, snapshot.age_ms)) {
            return snapshot;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return {};
}

std::string_view mavsdk_connection_error_message(MavsdkConnectionError error) {
    switch (error) {
    case MavsdkConnectionError::None:
        return "none";
    case MavsdkConnectionError::InvalidConfiguration:
        return "invalid MAVSDK endpoint or expected system ID";
    case MavsdkConnectionError::AddConnectionFailed:
        return "MAVSDK connection failed";
    case MavsdkConnectionError::DiscoveryTimeout:
        return "timed out waiting for expected MAVSDK autopilot heartbeat";
    case MavsdkConnectionError::WrongPeer:
        return "wrong autopilot peer";
    case MavsdkConnectionError::AmbiguousPeers:
        return "ambiguous autopilot peers";
    }
    return "unknown MAVSDK connection error";
}

} // namespace nomad::mavlink
