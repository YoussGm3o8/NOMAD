// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#include "nomad/mavlink/mavsdk_transport.hpp"

namespace nomad::mavlink {

// The default core build does not link MAVSDK; callers must treat a null return
// as "transport unavailable in this build".
std::unique_ptr<MavlinkConnection> make_mavsdk_connection(const std::string &, std::uint8_t,
                                                          std::chrono::milliseconds) {
    return nullptr;
}

} // namespace nomad::mavlink
