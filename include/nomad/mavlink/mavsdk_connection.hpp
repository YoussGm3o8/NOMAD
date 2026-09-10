// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

#pragma once

#include "nomad/mavlink/mavsdk_validation.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace nomad::mavlink {

enum class MavsdkConnectionError {
    None,
    InvalidConfiguration,
    AddConnectionFailed,
    DiscoveryTimeout,
    WrongPeer,
    AmbiguousPeers,
};

struct MavsdkConnectionOptions {
    std::string endpoint;
    std::uint8_t expected_system_id{};
    std::chrono::milliseconds discovery_timeout{5000};
};

struct MavsdkStatusSnapshot {
    mavsdk_phase_a::StatusValues values;
    std::size_t position_updates{};
    std::int64_t observation_ms{};
    std::int64_t age_ms{};
};

class MavsdkConnection {
  public:
    explicit MavsdkConnection(MavsdkConnectionOptions options);
    ~MavsdkConnection();

    MavsdkConnection(const MavsdkConnection &) = delete;
    MavsdkConnection &operator=(const MavsdkConnection &) = delete;

    bool connect();
    void disconnect();
    bool is_connected() const;
    std::uint8_t system_id() const;
    MavsdkConnectionError last_error() const;
    MavsdkStatusSnapshot get_status() const;
    std::optional<MavsdkStatusSnapshot> wait_for_status(std::chrono::milliseconds timeout) const;

  private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

std::string_view mavsdk_connection_error_message(MavsdkConnectionError error);

} // namespace nomad::mavlink
