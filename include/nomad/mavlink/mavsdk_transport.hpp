// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace nomad::mavlink {

// Creates a MAVSDK-backed MavlinkConnection for the given UDP endpoint and
// expected autopilot system ID. Returns nullptr in a build without the MAVSDK
// transport (NOMAD_ENABLE_MAVSDK=OFF) so the caller reports it as unavailable
// rather than silently falling back to another transport.
std::unique_ptr<MavlinkConnection>
make_mavsdk_connection(const std::string &endpoint, std::uint8_t expected_system_id,
                       std::chrono::milliseconds discovery_timeout);

} // namespace nomad::mavlink
