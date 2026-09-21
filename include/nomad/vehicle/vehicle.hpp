// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "nomad/mavlink/connection.hpp"
#include "nomad/safety/geofence.hpp"
#include "nomad/safety/payload.hpp"
#include "nomad/safety/velocity.hpp"
#include "nomad/safety/watchdog.hpp"
#include "nomad/vehicle/operation.hpp"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace nomad::vehicle {

// Altitude is above home (relative to takeoff point), matching the GCS
// guided-target conventions; takeoff uses the same relative frame.
struct Location {
    double latitude_deg{};
    double longitude_deg{};
    float altitude_m{};
};

struct CommandResult {
    bool success{false};
    std::string message;
};

class Vehicle {
  public:
    explicit Vehicle(mavlink::MavlinkConnection &connection, safety::WatchdogPolicy watchdog_policy = {},
                     safety::GlobalFencePolicy fence_policy = {}, safety::VelocityLimits velocity_limits = {},
                     std::chrono::milliseconds position_freshness_timeout = std::chrono::milliseconds(2000));
    ~Vehicle();

    Vehicle(const Vehicle &) = delete;
    Vehicle &operator=(const Vehicle &) = delete;

    std::optional<telemetry::VehicleState> wait_for_state(std::chrono::milliseconds timeout);

    // Polls for up to @p timeout and returns the freshest state that satisfies
    // @p is_complete. When none does, returns the freshest state seen at all so
    // a caller can still report partial telemetry; nullopt means no telemetry
    // arrived inside the window.
    std::optional<telemetry::VehicleState>
    wait_for_telemetry(std::chrono::milliseconds timeout,
                       const std::function<bool(const telemetry::VehicleState &)> &is_complete);

    CommandResult arm();
    CommandResult disarm();
    CommandResult set_mode(std::uint32_t custom_mode);
    CommandResult set_guided_mode();
    CommandResult takeoff(float altitude_m);
    CommandResult vtol_takeoff(float altitude_m);
    CommandResult update_vio(bool healthy, float confidence);
    CommandResult set_velocity(const safety::VelocityCommand &command);
    CommandResult set_servo(int channel, int pwm_microseconds);
    CommandResult set_relay(int relay_number, bool on);
    CommandResult motor_test(int motor_instance, int pwm_microseconds, float timeout_seconds);
    CommandResult configure_gimbal(int mount_mode);
    CommandResult send_user_command(const std::array<float, 7> &parameters);
    CommandResult arm_payload();
    CommandResult release_payload(int relay_number, float duration_seconds);
    CommandResult stop_velocity();
    bool velocity_control_active() const;
    safety::WatchdogReason last_velocity_stop_reason() const;
    CommandResult goto_location(const Location &location);
    CommandResult land();
    CommandResult wait_until_disarmed(std::chrono::milliseconds timeout);
    CommandResult return_to_launch();
    CommandResult upload_fence(const std::vector<safety::GlobalPoint> &boundary);
    CommandResult verify_fence_uploaded(const std::vector<safety::GlobalPoint> &expected_boundary);

  private:
    // A waiter's verdict: nullopt keeps polling, a result stops the wait.
    using StateVerdict = std::function<std::optional<CommandResult>(const telemetry::VehicleState &)>;

    CommandResult send_command(const mavlink::Command &command, const char *name);
    CommandResult wait_for_state_until(std::chrono::milliseconds timeout, std::string timeout_message,
                                       const StateVerdict &verdict);
    CommandResult wait_for_armed_state(bool expected, const char *name);
    CommandResult wait_for_mode(std::uint32_t expected, const char *name);
    CommandResult wait_for_mode(const std::function<bool(std::uint32_t)> &matches, const char *name);
    CommandResult send_mode_and_verify(std::uint32_t custom_mode, const char *name);
    CommandResult wait_for_altitude(float minimum_altitude_m, const char *name);
    CommandResult wait_for_vtol_takeoff(float target_altitude_m);
    CommandResult wait_for_location(const Location &location);
    CommandResult require_operation(VehicleOperation operation) const;
    // A fresh heartbeat does not imply a fresh position: callers fail closed
    // when this is true rather than trusting a stale fix.
    bool position_is_stale(const telemetry::VehicleState &state) const;
    safety::FlightConditions get_flight_conditions(const telemetry::VehicleState &state,
                                                   std::chrono::steady_clock::time_point now) const;
    safety::WatchdogInput get_watchdog_input(const telemetry::VehicleState &state,
                                             std::chrono::steady_clock::time_point now) const;
    void start_watchdog_locked();
    void run_watchdog();
    bool send_zero_velocity_locked(safety::WatchdogReason reason);

    mavlink::MavlinkConnection &connection_;
    safety::WatchdogPolicy watchdog_policy_;
    safety::GlobalFencePolicy fence_policy_;
    safety::VelocityLimits velocity_limits_;
    std::chrono::milliseconds position_freshness_timeout_{std::chrono::milliseconds(2000)};
    safety::ReleaseInterlock payload_interlock_;
    mutable std::mutex payload_mutex_;
    mutable std::mutex velocity_mutex_;
    std::condition_variable velocity_condition_;
    std::thread watchdog_thread_;
    bool shutting_down_{false};
    bool velocity_control_active_{false};
    safety::WatchdogReason last_velocity_stop_reason_{safety::WatchdogReason::none};
    std::chrono::steady_clock::time_point last_command_time_{};
    std::chrono::steady_clock::time_point last_vio_update_{};
    bool vio_healthy_{false};
    float vio_confidence_{0.0F};
};

} // namespace nomad::vehicle
