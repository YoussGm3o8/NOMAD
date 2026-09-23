// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"
#include "nomad/safety/geofence.hpp"
#include "nomad/safety/velocity.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace nomad::runtime {

struct RuntimeConfig {
    std::uint16_t ipc_port{14611};
    std::chrono::milliseconds discovery_timeout{std::chrono::seconds(6)};
    std::chrono::milliseconds reconnect_delay{std::chrono::seconds(1)};
    std::string version{"0.1.0"};
    safety::GlobalFencePolicy fence_policy{};
    safety::VelocityLimits velocity_limits{};
    bool actuation_enabled{false};
};

// Owns one MAVLink connection and one Vehicle for the life of the process.
// Clients communicate through the versioned loopback IPC endpoint.
class Runtime {
  public:
    Runtime(std::unique_ptr<mavlink::MavlinkConnection> connection, RuntimeConfig config = {});
    ~Runtime();

    Runtime(const Runtime &) = delete;
    Runtime &operator=(const Runtime &) = delete;

    bool start(std::string &error);
    void stop();
    bool ready() const;

  private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace nomad::runtime
