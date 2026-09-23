// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/telemetry/identity.hpp"

#include <chrono>
#include <cstdint>
#include <string_view>

namespace nomad::telemetry {

struct Position {
    double latitude_deg{};
    double longitude_deg{};
    float altitude_m{};
    float relative_altitude_m{};
};

struct Velocity {
    float north_mps{};
    float east_mps{};
    float down_mps{};
    float groundspeed_mps{};
    float climb_rate_mps{};
};

struct Attitude {
    float roll_deg{};
    float pitch_deg{};
    float yaw_deg{};
};

struct Battery {
    float voltage_v{};
    float remaining_percent{};
};

struct Gps {
    std::uint8_t fix_type{};
    std::uint8_t satellites{};
};

// These values mirror MAV_VTOL_STATE from the pinned MAVLink dialect. Keep
// them in the telemetry boundary so the vehicle layer does not depend on
// generated MAVSDK or MAVLink headers.
enum class VtolState : std::uint8_t {
    Undefined = 0,
    TransitionToFixedWing = 1,
    TransitionToMulticopter = 2,
    Multicopter = 3,
    FixedWing = 4,
};

inline std::string_view vtol_state_name(VtolState state) {
    switch (state) {
    case VtolState::Undefined:
        return "undefined";
    case VtolState::TransitionToFixedWing:
        return "transition_to_fixed_wing";
    case VtolState::TransitionToMulticopter:
        return "transition_to_multicopter";
    case VtolState::Multicopter:
        return "multicopter";
    case VtolState::FixedWing:
        return "fixed_wing";
    }
    return "undefined";
}

struct VehicleState {
    bool connected{false};
    bool heartbeat_fresh{false};
    bool armed{false};
    std::uint8_t system_id{};
    std::uint8_t component_id{};
    std::uint32_t custom_mode{0};
    Position position{};
    Velocity velocity{};
    Attitude attitude{};
    Battery battery{};
    Gps gps{};
    bool position_valid{false};
    bool battery_valid{false};
    bool gps_valid{false};
    bool attitude_valid{false};
    VtolState vtol_state{VtolState::Undefined};
    bool vtol_state_valid{false};
    std::chrono::steady_clock::time_point position_updated_at{};
    std::chrono::steady_clock::time_point battery_updated_at{};
    std::chrono::steady_clock::time_point gps_updated_at{};
    std::chrono::steady_clock::time_point attitude_updated_at{};
    std::chrono::steady_clock::time_point vtol_state_updated_at{};
    VehicleIdentity identity{};
};

}  // namespace nomad::telemetry
