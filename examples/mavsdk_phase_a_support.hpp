// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace nomad::mavsdk_phase_a {

enum class SystemSelection {
    Selected,
    NoAutopilot,
    WrongPeer,
    Ambiguous,
};

struct StatusValues {
    double latitude_deg{};
    double longitude_deg{};
    double relative_altitude_m{};
    int gps_fix_type{};
    int satellites{};
    double battery_voltage_v{};
    double battery_remaining_percent{};
    int flight_mode{};
};

std::optional<std::string> canonicalize_udp_endpoint(std::string_view endpoint);
std::optional<std::uint8_t> parse_system_id(std::string_view value);
SystemSelection classify_system_ids(const std::vector<std::uint8_t>& system_ids, std::uint8_t expected_id);
bool has_valid_status(const StatusValues& values);
bool has_fresh_position_stream(std::size_t update_count, std::int64_t observation_ms, std::int64_t age_ms);

} // namespace nomad::mavsdk_phase_a
