// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class FakeConnection final : public nomad::mavlink::MavlinkConnection {
  public:
    FakeConnection() {
        state->identity = nomad::telemetry::identify_vehicle(nomad::telemetry::kArduPilotAutopilot,
                                                              nomad::telemetry::kQuadrotor);
    }

    struct GotoRequest {
        double latitude_deg{};
        double longitude_deg{};
        float relative_altitude_m{};
    };

    bool connect() override {
        connected = true;
        return true;
    }

    void disconnect() override {
        event_log.push_back("disconnect");
        connected = false;
    }

    bool is_connected() const override {
        return connected;
    }

    nomad::mavlink::ConnectFailure get_connect_failure() const override {
        return nomad::mavlink::ConnectFailure::None;
    }

    std::optional<nomad::mavlink::Heartbeat> wait_for_heartbeat(std::chrono::milliseconds) override {
        return nomad::mavlink::Heartbeat{1, 1, 0, 2, 3, 0};
    }

    std::optional<nomad::telemetry::VehicleState> wait_for_state(std::chrono::milliseconds) override {
        if (auto_stamp_fresh_fields) {
            stamp_fresh_fields();
        }
        return get_state();
    }

    nomad::telemetry::VehicleState get_state() const override {
        std::lock_guard lock(state_mutex);
        return *state;
    }

    std::optional<nomad::mavlink::CommandAck> send_command(const nomad::mavlink::Command &command,
                                                           std::chrono::milliseconds) override {
        std::lock_guard lock(state_mutex);
        last_command = command;
        command_history.push_back(command);
        update_state_for_command(command);
        if (!command_send_results.empty()) {
            const bool result = command_send_results.front();
            command_send_results.erase(command_send_results.begin());
            if (!result) {
                return std::nullopt;
            }
        }
        if (!acknowledgement.has_value()) {
            return std::nullopt;
        }
        return nomad::mavlink::CommandAck{command.id, acknowledgement->result};
    }

    bool goto_location_relative(double latitude_deg, double longitude_deg, float relative_altitude_m,
                                std::chrono::milliseconds) override {
        std::lock_guard lock(state_mutex);
        last_goto = GotoRequest{latitude_deg, longitude_deg, relative_altitude_m};
        if (!acknowledgement || acknowledgement->command != 192 || acknowledgement->result != 0) {
            return false;
        }
        state->position_valid = true;
        state->position.latitude_deg = latitude_deg;
        state->position.longitude_deg = longitude_deg;
        state->position.relative_altitude_m = relative_altitude_m;
        return true;
    }

    bool send_velocity(const nomad::mavlink::VelocitySetpoint &setpoint) override {
        event_log.push_back("send_velocity");
        last_velocity = setpoint;
        velocity_send_count += 1;
        if (!velocity_send_results.empty()) {
            const bool result = velocity_send_results.front();
            velocity_send_results.erase(velocity_send_results.begin());
            return result;
        }
        return true;
    }

    bool is_velocity_active() const override {
        return last_velocity.vx != 0.0F || last_velocity.vy != 0.0F || last_velocity.vz != 0.0F ||
               last_velocity.yaw_rate != 0.0F;
    }

    bool send_fence_point(const nomad::mavlink::FencePoint &point, std::uint8_t index, std::uint8_t total) override {
        fence_points.push_back(point);
        fence_indices.push_back(index);
        fence_total = total;
        return fence_send_results.empty() || take_fence_send_result();
    }

    bool request_fence_point(std::uint8_t index) override {
        requested_fence_indices.push_back(index);
        return true;
    }

    bool upload_fence_plan(const std::vector<nomad::mavlink::FencePlanItem> &items) override {
        fence_plan_upload_count += 1;
        uploaded_fence_plan = items;
        return fence_plan_upload_result;
    }

    std::optional<std::vector<nomad::mavlink::FencePlanItem>> download_fence_plan(std::chrono::milliseconds) override {
        fence_plan_download_count += 1;
        if (!fence_plan_download_result) {
            return std::nullopt;
        }
        return uploaded_fence_plan;
    }

    std::optional<nomad::mavlink::FencePoint> wait_for_fence_point(std::chrono::milliseconds) override {
        if (readback_index >= fence_points.size()) {
            return std::nullopt;
        }
        return fence_points[readback_index++];
    }

    std::optional<float> read_param(const std::string &param_id, std::chrono::milliseconds) override {
        parameter_read_count += 1;
        const auto found = parameters.find(param_id);
        if (found == parameters.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    bool connected{false};
    // Most tests model a live telemetry feed, so polling refreshes the sample
    // timestamps by default. Tests for stale-feed behavior clear this flag and
    // set the relevant *_updated_at by hand.
    bool auto_stamp_fresh_fields{true};
    std::optional<nomad::telemetry::VehicleState> state{
        nomad::telemetry::VehicleState{true, true, false, 1, 1, 4},
    };
    nomad::mavlink::VelocitySetpoint last_velocity{};
    int velocity_send_count{0};
    nomad::mavlink::Command last_command{};
    std::optional<GotoRequest> last_goto;
    std::optional<nomad::mavlink::CommandAck> acknowledgement{
        nomad::mavlink::CommandAck{0, 0},
    };
    std::vector<nomad::mavlink::Command> command_history;
    std::vector<bool> velocity_send_results;
    std::vector<bool> command_send_results;
    std::vector<nomad::mavlink::FencePoint> fence_points;
    std::vector<std::uint8_t> fence_indices;
    std::uint8_t fence_total{0};
    std::vector<std::uint8_t> requested_fence_indices;
    std::size_t readback_index{0};
    std::vector<bool> fence_send_results;
    std::vector<nomad::mavlink::FencePlanItem> uploaded_fence_plan;
    bool fence_plan_upload_result{true};
    bool fence_plan_download_result{true};
    int fence_plan_upload_count{0};
    int fence_plan_download_count{0};
    int parameter_read_count{0};
    std::map<std::string, float> parameters;
    std::vector<std::string> event_log;

    void set_connected(bool value) {
        std::lock_guard lock(state_mutex);
        state->connected = value;
        state->heartbeat_fresh = value;
    }

    void set_mode(std::uint32_t mode) {
        std::lock_guard lock(state_mutex);
        state->custom_mode = mode;
    }

  private:
    mutable std::mutex state_mutex;

    void stamp_fresh_fields() {
        std::lock_guard lock(state_mutex);
        const auto now = std::chrono::steady_clock::now();
        if (state->position_valid) {
            state->position_updated_at = now;
        }
        if (state->battery_valid) {
            state->battery_updated_at = now;
        }
        if (state->gps_valid) {
            state->gps_updated_at = now;
        }
        if (state->attitude_valid) {
            state->attitude_updated_at = now;
        }
    }

    bool take_fence_send_result() {
        const bool result = fence_send_results.front();
        fence_send_results.erase(fence_send_results.begin());
        return result;
    }

    void update_state_for_command(const nomad::mavlink::Command &command) {
        if (command.id == 400) {
            state->armed = command.parameters[0] > 0.0F;
        } else if (command.id == 176) {
            state->custom_mode = static_cast<std::uint32_t>(command.parameters[1]);
        } else if (command.id == 22) {
            state->position_valid = true;
            state->position.relative_altitude_m = command.parameters[6];
        } else if (command.id == 21) {
            if (state->identity.aircraft_class == nomad::telemetry::AircraftClass::Copter) {
                state->custom_mode = 9;
                state->armed = false;
            }
        } else if (command.id == 20) {
            switch (state->identity.aircraft_class) {
            case nomad::telemetry::AircraftClass::Copter:
                state->custom_mode = 6;
                break;
            case nomad::telemetry::AircraftClass::Plane:
                state->custom_mode = 11;
                break;
            case nomad::telemetry::AircraftClass::QuadPlane:
                state->custom_mode = 21;
                break;
            case nomad::telemetry::AircraftClass::Unknown:
                break;
            }
        }
    }
};
