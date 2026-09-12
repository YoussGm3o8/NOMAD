// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
//
// MAVSDK-backed MavlinkConnection: the Phase B transport. MAVSDK owns framing,
// transport and its internal workers; NOMAD keeps command semantics and
// outcome verification. Commands are issued as raw COMMAND_LONG/COMMAND_INT so
// the existing NOMAD contract (MAVLink result code, relative-altitude frame,
// every verb ArduPilot accepts) is preserved exactly.

#include "mavsdk_mavlink_connection.hpp"

#include "mavsdk_system.hpp"
#include "nomad/mavlink/mavsdk_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nomad::mavlink {
namespace {

constexpr double kPositionScale = 1e7;
constexpr auto kTelemetryWaitIncrement = std::chrono::milliseconds(20);

// The velocity command is SET_POSITION_TARGET_LOCAL_NED in the body frame with
// position, acceleration and absolute-yaw ignored, exactly as the legacy codec
// built it: the vehicle must see one frame whichever transport sends it.
constexpr std::uint16_t kVelocityTypeMask = 0x07c7;
constexpr std::uint8_t kBodyOffsetNedFrame = 9; // MAV_FRAME_BODY_OFFSET_NED
constexpr std::uint32_t kIgnoredTimestamp = 0;

bool is_zero_setpoint(const VelocitySetpoint &setpoint) {
    return setpoint.vx == 0.0F && setpoint.vy == 0.0F && setpoint.vz == 0.0F && setpoint.yaw_rate == 0.0F;
}

bool has_finite_components(const VelocitySetpoint &setpoint) {
    return std::isfinite(setpoint.vx) && std::isfinite(setpoint.vy) && std::isfinite(setpoint.vz) &&
           std::isfinite(setpoint.yaw_rate);
}

bool has_telemetry(const telemetry::VehicleState &state) {
    return state.position_valid || state.battery_valid || state.gps_valid || state.attitude_valid;
}

bool has_configuration(const std::string &endpoint, std::uint8_t expected_system_id,
                       std::chrono::milliseconds discovery_timeout) {
    return mavsdk_phase_a::canonicalize_udp_endpoint(endpoint).has_value() && expected_system_id != 0 &&
           discovery_timeout > std::chrono::milliseconds::zero();
}

// MAV_RESULT codes NOMAD reports for a completed command. MAVSDK exposes the
// classified result rather than the raw code; timeout and link errors are not
// acknowledgements at all and must surface as "no ack" so NOMAD fails closed.
std::optional<std::uint8_t> command_result_code(mavsdk::MavlinkPassthrough::Result result) {
    using Result = mavsdk::MavlinkPassthrough::Result;
    switch (result) {
    case Result::Success:
        return 0; // MAV_RESULT_ACCEPTED
    case Result::CommandTemporarilyRejected:
    case Result::CommandBusy:
        return 1; // MAV_RESULT_TEMPORARILY_REJECTED
    case Result::CommandDenied:
        return 2; // MAV_RESULT_DENIED
    case Result::CommandUnsupported:
        return 3; // MAV_RESULT_UNSUPPORTED
    case Result::CommandFailed:
        return 4; // MAV_RESULT_FAILED
    default:
        return std::nullopt;
    }
}

} // namespace

MavsdkMavlinkConnection::MavsdkMavlinkConnection(std::string endpoint, std::uint8_t expected_system_id,
                                                 std::chrono::milliseconds discovery_timeout)
    : endpoint_(std::move(endpoint)),
      expected_system_id_(expected_system_id),
      discovery_timeout_(discovery_timeout),
      sdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}) {}

MavsdkMavlinkConnection::~MavsdkMavlinkConnection() {
    disconnect();
}

bool MavsdkMavlinkConnection::connect() {
    if (is_connected()) {
        return true;
    }
    close();
    if (!has_configuration(endpoint_, expected_system_id_, discovery_timeout_)) {
        return false;
    }
    const auto endpoint = mavsdk_phase_a::canonicalize_udp_endpoint(endpoint_);
    auto [result, handle] = sdk_.add_any_connection_with_handle(*endpoint);
    if (result != mavsdk::ConnectionResult::Success) {
        return false;
    }
    handle_ = handle;
    const auto deadline = ObservationClock::now() + discovery_timeout_;
    while (ObservationClock::now() < deadline) {
        if (select_system()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    close();
    return false;
}

void MavsdkMavlinkConnection::disconnect() {
    // Zero the vehicle while the target is still latched, then tear the link
    // down. The legacy transport zeros here for the same reason: the last
    // setpoint it sent would otherwise keep steering a vehicle NOMAD has
    // stopped controlling.
    if (is_velocity_active()) {
        send_velocity({});
    }
    close();
}

bool MavsdkMavlinkConnection::is_connected() const {
    return system_ != nullptr && system_->is_connected();
}

bool MavsdkMavlinkConnection::select_system() {
    mavsdk_phase_a::SystemSelection selection{};
    const auto candidate = mavsdk_system::select_expected_autopilot(sdk_, expected_system_id_, selection);
    if (candidate == nullptr) {
        return false;
    }
    system_ = candidate;
    target_system_ = system_->get_system_id();
    target_component_ = kAutopilotComponent;
    subscribe();
    return true;
}

void MavsdkMavlinkConnection::subscribe() {
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    passthrough_ = std::make_unique<mavsdk::MavlinkPassthrough>(system_);
    geofence_ = std::make_unique<mavsdk::Geofence>(system_);
    param_ = std::make_unique<mavsdk::Param>(system_);
    position_handle_ = telemetry_->subscribe_position([this](const auto &value) { observe_position(value); });
    velocity_handle_ = telemetry_->subscribe_velocity_ned([this](const auto &value) { observe_velocity(value); });
    battery_handle_ = telemetry_->subscribe_battery([this](const auto &value) { observe_battery(value); });
    gps_handle_ = telemetry_->subscribe_gps_info([this](const auto &value) { observe_gps(value); });
    attitude_handle_ = telemetry_->subscribe_attitude_euler([this](const auto &value) { observe_attitude(value); });
    heartbeat_handle_ = passthrough_->subscribe_message(MAVLINK_MSG_ID_HEARTBEAT,
                                                        [this](const auto &message) { observe_heartbeat(message); });
}

void MavsdkMavlinkConnection::unsubscribe() {
    if (telemetry_) {
        if (position_handle_) {
            telemetry_->unsubscribe_position(*position_handle_);
        }
        if (velocity_handle_) {
            telemetry_->unsubscribe_velocity_ned(*velocity_handle_);
        }
        if (battery_handle_) {
            telemetry_->unsubscribe_battery(*battery_handle_);
        }
        if (gps_handle_) {
            telemetry_->unsubscribe_gps_info(*gps_handle_);
        }
        if (attitude_handle_) {
            telemetry_->unsubscribe_attitude_euler(*attitude_handle_);
        }
    }
    if (passthrough_ && heartbeat_handle_) {
        passthrough_->unsubscribe_message(MAVLINK_MSG_ID_HEARTBEAT, *heartbeat_handle_);
    }
    position_handle_.reset();
    velocity_handle_.reset();
    battery_handle_.reset();
    gps_handle_.reset();
    attitude_handle_.reset();
    heartbeat_handle_.reset();
}

void MavsdkMavlinkConnection::close() {
    unsubscribe();
    telemetry_.reset();
    passthrough_.reset();
    geofence_.reset();
    param_.reset();
    system_.reset();
    if (handle_) {
        sdk_.remove_connection(*handle_);
        handle_.reset();
    }
    std::lock_guard lock(observation_mutex_);
    state_ = {};
    heartbeat_.reset();
    last_heartbeat_ = {};
    velocity_active_ = false;
}

void MavsdkMavlinkConnection::observe_heartbeat(const mavlink_message_t &message) {
    mavlink_heartbeat_t decoded{};
    mavlink_msg_heartbeat_decode(&message, &decoded);
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.connected = true;
    state_.heartbeat_fresh = true;
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    state_.custom_mode = decoded.custom_mode;
    state_.armed = (decoded.base_mode & kArmModeFlag) != 0;
    heartbeat_ = Heartbeat{message.sysid, message.compid, decoded.custom_mode, decoded.type, decoded.autopilot,
                           decoded.base_mode};
    last_heartbeat_ = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_position(const mavsdk::Telemetry::Position &position) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.position.latitude_deg = position.latitude_deg;
    state_.position.longitude_deg = position.longitude_deg;
    state_.position.altitude_m = position.absolute_altitude_m;
    state_.position.relative_altitude_m = position.relative_altitude_m;
    state_.position_valid = true;
    state_.position_updated_at = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_velocity(const mavsdk::Telemetry::VelocityNed &velocity) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.velocity.north_mps = velocity.north_m_s;
    state_.velocity.east_mps = velocity.east_m_s;
    state_.velocity.down_mps = velocity.down_m_s;
    state_.velocity.groundspeed_mps = std::sqrt(velocity.north_m_s * velocity.north_m_s +
                                                velocity.east_m_s * velocity.east_m_s);
}

void MavsdkMavlinkConnection::observe_battery(const mavsdk::Telemetry::Battery &battery) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.battery.voltage_v = battery.voltage_v;
    state_.battery.remaining_percent = battery.remaining_percent;
    state_.battery_valid = true;
    state_.battery_updated_at = ObservationClock::now();
}

void MavsdkMavlinkConnection::observe_gps(const mavsdk::Telemetry::GpsInfo &gps) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.gps.fix_type = static_cast<std::uint8_t>(gps.fix_type);
    state_.gps.satellites = static_cast<std::uint8_t>(gps.num_satellites);
    state_.gps_valid = gps.fix_type != mavsdk::Telemetry::FixType::NoGps;
    if (state_.gps_valid) {
        state_.gps_updated_at = ObservationClock::now();
    }
}

void MavsdkMavlinkConnection::observe_attitude(const mavsdk::Telemetry::EulerAngle &attitude) {
    ObservationUpdate update(observation_mutex_, observation_changed_);
    state_.attitude.roll_deg = attitude.roll_deg;
    state_.attitude.pitch_deg = attitude.pitch_deg;
    state_.attitude.yaw_deg = attitude.yaw_deg;
    state_.attitude_valid = true;
    state_.attitude_updated_at = ObservationClock::now();
}

telemetry::VehicleState MavsdkMavlinkConnection::state_locked() const {
    auto state = state_;
    if (last_heartbeat_ == ObservationClock::time_point{}) {
        state.heartbeat_fresh = false;
        state.connected = false;
        return state;
    }
    const auto elapsed = ObservationClock::now() - last_heartbeat_;
    state.heartbeat_fresh = elapsed <= kHeartbeatTimeout;
    state.connected = state.heartbeat_fresh;
    return state;
}

telemetry::VehicleState MavsdkMavlinkConnection::get_state() const {
    std::lock_guard lock(observation_mutex_);
    return state_locked();
}

std::optional<Heartbeat> MavsdkMavlinkConnection::wait_for_heartbeat(std::chrono::milliseconds timeout) {
    std::unique_lock lock(observation_mutex_);
    const auto deadline = ObservationClock::now() + timeout;
    while (ObservationClock::now() < deadline) {
        if (heartbeat_.has_value() && state_locked().connected) {
            return heartbeat_;
        }
        observation_changed_.wait_until(lock, (std::min)(deadline, ObservationClock::now() + kTelemetryWaitIncrement));
    }
    return std::nullopt;
}

std::optional<telemetry::VehicleState> MavsdkMavlinkConnection::wait_for_state(std::chrono::milliseconds timeout) {
    std::unique_lock lock(observation_mutex_);
    const auto deadline = ObservationClock::now() + timeout;
    while (ObservationClock::now() < deadline) {
        const auto state = state_locked();
        if (state.connected && has_telemetry(state)) {
            return state;
        }
        observation_changed_.wait_until(lock, (std::min)(deadline, ObservationClock::now() + kTelemetryWaitIncrement));
    }
    return std::nullopt;
}

mavsdk::MavlinkPassthrough::Result MavsdkMavlinkConnection::send_long(const Command &command) {
    mavsdk::MavlinkPassthrough::CommandLong wire{};
    wire.target_sysid = target_system_;
    wire.target_compid = target_component_;
    wire.command = command.id;
    wire.param1 = command.parameters[0];
    wire.param2 = command.parameters[1];
    wire.param3 = command.parameters[2];
    wire.param4 = command.parameters[3];
    wire.param5 = command.parameters[4];
    wire.param6 = command.parameters[5];
    wire.param7 = command.parameters[6];
    return passthrough_->send_command_long(wire);
}

mavsdk::MavlinkPassthrough::Result MavsdkMavlinkConnection::send_int(const Command &command) {
    // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT with 1e7-scaled integer degrees and
    // above-home altitude, matching the legacy codec exactly.
    mavsdk::MavlinkPassthrough::CommandInt wire{};
    wire.target_sysid = target_system_;
    wire.target_compid = target_component_;
    wire.command = command.id;
    wire.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    wire.param1 = command.parameters[0];
    wire.param2 = command.parameters[1];
    wire.param3 = command.parameters[2];
    wire.param4 = command.parameters[3];
    wire.x = static_cast<std::int32_t>(static_cast<double>(command.parameters[4]) * kPositionScale);
    wire.y = static_cast<std::int32_t>(static_cast<double>(command.parameters[5]) * kPositionScale);
    wire.z = command.parameters[6];
    return passthrough_->send_command_int(wire);
}

mavsdk::MavlinkPassthrough::Result
MavsdkMavlinkConnection::queue_velocity_setpoint(const VelocitySetpoint &setpoint) {
    // queue_message() is MAVSDK's supported send path: it owns the sequence
    // numbering and hands back the address to pack with.
    // MavlinkAddress is a plain struct in the global namespace of the MAVSDK
    // header, which is why it is not qualified here.
    return passthrough_->queue_message([&](MavlinkAddress address, std::uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_set_position_target_local_ned_pack_chan(
            address.system_id, address.component_id, channel, &message, kIgnoredTimestamp, target_system_,
            target_component_, kBodyOffsetNedFrame, kVelocityTypeMask, 0.0F, 0.0F, 0.0F, setpoint.vx, setpoint.vy,
            setpoint.vz, 0.0F, 0.0F, 0.0F, 0.0F, setpoint.yaw_rate);
        return message;
    });
}

bool MavsdkMavlinkConnection::send_velocity(const VelocitySetpoint &setpoint) {
    if (!has_finite_components(setpoint) || !passthrough_ || target_system_ == 0) {
        return false;
    }
    // A non-zero setpoint needs a live, latched peer. A zero setpoint is the
    // safety command the watchdog and shutdown paths rely on, so it is allowed
    // out on a link the core already believes is dead.
    const bool is_zero = is_zero_setpoint(setpoint);
    if (!is_zero && (!is_connected() || !get_state().connected)) {
        return false;
    }
    if (queue_velocity_setpoint(setpoint) != mavsdk::MavlinkPassthrough::Result::Success) {
        return false;
    }
    std::lock_guard lock(observation_mutex_);
    velocity_active_ = !is_zero;
    return true;
}

bool MavsdkMavlinkConnection::is_velocity_active() const {
    std::lock_guard lock(observation_mutex_);
    return velocity_active_;
}

std::optional<float> MavsdkMavlinkConnection::read_param(const std::string &param_id,
                                                         std::chrono::milliseconds /*timeout*/) {
    // MAVSDK's blocking read applies its own request timeout, so the caller's
    // budget is not stacked on top of it. A read either returns the autopilot's
    // value or fails closed; nothing is inferred from a missing answer.
    if (!is_connected() || !param_ || param_id.empty()) {
        return std::nullopt;
    }
    const auto [result, value] = param_->get_param_float(param_id);
    if (result != mavsdk::Param::Result::Success) {
        return std::nullopt;
    }
    return value;
}

std::optional<CommandAck> MavsdkMavlinkConnection::send_command(const Command &command,
                                                                std::chrono::milliseconds /*timeout*/) {
    if (!is_connected() || !passthrough_) {
        return std::nullopt;
    }
    const auto result = command.use_command_int ? send_int(command) : send_long(command);
    const auto code = command_result_code(result);
    if (!code.has_value()) {
        return std::nullopt;
    }
    return CommandAck{command.id, *code};
}

// MAVSDK's telemetry plugin owns stream/interval setup for the subscriptions
// made in subscribe(); there is no raw REQUEST_DATA_STREAM to send. No
// production caller asks for a stream (run_status documents why), so this
// reports the transport is available rather than claiming a frame was sent.
mavsdk::MavlinkPassthrough::Result
MavsdkMavlinkConnection::queue_data_stream_request(std::uint8_t stream_id, std::uint16_t message_rate) {
    // start_stop = 1 matches the legacy codec's request. MAVSDK's own Telemetry
    // plugin subscribes to what it needs, but a caller that asks for a stream
    // must still have the frame put on the wire rather than be told it was.
    return passthrough_->queue_message([&](MavlinkAddress address, std::uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_request_data_stream_pack_chan(address.system_id, address.component_id, channel, &message,
                                                 target_system_, target_component_, stream_id, message_rate, 1);
        return message;
    });
}

bool MavsdkMavlinkConnection::request_data_stream(std::uint8_t stream_id, std::uint16_t message_rate) {
    // A stream request needs a live, latched peer, the same gate the legacy
    // transport applied. Answering from the connection's own state would report
    // success for a request that was never sent.
    if (!is_connected() || !passthrough_ || target_system_ == 0 || !get_state().connected) {
        return false;
    }
    return queue_data_stream_request(stream_id, message_rate) == mavsdk::MavlinkPassthrough::Result::Success;
}

std::unique_ptr<MavlinkConnection> make_mavsdk_connection(const std::string &endpoint,
                                                          std::uint8_t expected_system_id,
                                                          std::chrono::milliseconds discovery_timeout) {
    return std::make_unique<MavsdkMavlinkConnection>(endpoint, expected_system_id, discovery_timeout);
}

} // namespace nomad::mavlink
