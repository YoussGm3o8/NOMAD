// SPDX-License-Identifier: Apache-2.0
// Transport-level zero-delivery proof (SR-LNK-03): drive a live loopback UDP
// transport and prove the safety zero setpoint (SET_POSITION_TARGET_LOCAL_NED
// with zero rates) actually leaves the socket when the velocity watchdog
// stops, when the caller stops velocity control, when the transport
// disconnects, and when the vehicle is destroyed on a link the core already
// believes dead. Fake transports only record what the core asked for; these
// tests decode what was actually received on the wire.
#include "nomad/mavlink/protocol.hpp"
#include "nomad/mavlink/udp_connection.hpp"
#include "nomad/safety/watchdog.hpp"
#include "nomad/vehicle/vehicle.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket kInvalidSocket = INVALID_SOCKET;
#else
using Socket = int;
constexpr Socket kInvalidSocket = -1;
#endif

void close_socket(Socket socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

// A failing assert on Windows opens a dialog that blocks unattended CI runs,
// so the checks below throw instead of calling assert().
void check_impl(bool ok, const char *condition, int line) {
    if (!ok) {
        throw std::runtime_error(std::string("check failed at line ") + std::to_string(line) + ": " + condition);
    }
}

#define CHECK(condition) check_impl(static_cast<bool>(condition), #condition, __LINE__)

// Bind to an ephemeral loopback port, read the assigned number, and release
// it. The connection under test then binds that port.
std::uint16_t find_free_udp_port() {
    const Socket probe = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    CHECK(probe != kInvalidSocket);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    CHECK(bind(probe, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
    sockaddr_in bound{};
#ifdef _WIN32
    int length = sizeof(bound);
#else
    socklen_t length = sizeof(bound);
#endif
    CHECK(getsockname(probe, reinterpret_cast<sockaddr *>(&bound), &length) == 0);
    const auto port = ntohs(bound.sin_port);
    close_socket(probe);
    return port;
}

std::vector<std::uint8_t> vehicle_heartbeat_frame(std::uint8_t sequence) {
    std::vector<std::uint8_t> payload(9, 0);
    payload[0] = 4;  // custom_mode = 4 (GUIDED), little-endian
    payload[4] = 2;  // MAV_TYPE_QUADROTOR
    payload[5] = 3;  // MAV_AUTOPILOT_ARDUPILOTMEGA
    payload[6] = 0x81;  // base_mode: armed (0x80) + custom mode understood (0x01)
    payload[8] = 3;  // MAVLink protocol version
    return nomad::mavlink::encode_message(sequence, 1, 1, 0, payload, true).value();
}

// Vehicle stand-in on a loopback socket: records every datagram the core
// sends and, while live, answers each one with a valid vehicle heartbeat so
// the link stays fresh. It binds an ephemeral port and announces vehicle
// heartbeats to the connection's listen port until the core latches it as the
// peer (a listener latches the heartbeat's source address). set_silent(true)
// stops the replies while keeping the socket open — a vehicle whose heartbeat
// died without losing the delivery path, the worst case a link-loss zero must
// still cover.
class VehicleStandIn {
  public:
    explicit VehicleStandIn(std::uint16_t connection_port) : connection_port_(connection_port) {
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        CHECK(socket_ != kInvalidSocket);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;  // ephemeral: the core latches this address as its peer
        CHECK(bind(socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
        sockaddr_in bound{};
#ifdef _WIN32
        int length = sizeof(bound);
        DWORD receive_timeout_ms = 100;
        CHECK(setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&receive_timeout_ms),
                         sizeof(receive_timeout_ms)) == 0);
#else
        socklen_t length = sizeof(bound);
        timeval receive_timeout{};
        receive_timeout.tv_sec = 0;
        receive_timeout.tv_usec = 100 * 1000;
        CHECK(setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout)) == 0);
#endif
        CHECK(getsockname(socket_, reinterpret_cast<sockaddr *>(&bound), &length) == 0);
        port_ = ntohs(bound.sin_port);
        thread_ = std::thread([this] { run(); });
    }

    ~VehicleStandIn() {
        running_ = false;
        thread_.join();
        close_socket(socket_);
    }

    std::uint16_t port() const {
        return port_;
    }

    void set_silent(bool value) {
        silent_ = value;
    }

    std::vector<std::vector<std::uint8_t>> recorded() const {
        std::lock_guard lock(mutex_);
        return datagrams_;
    }

  private:
    void run() {
        std::uint8_t sequence = 0;
        std::array<std::uint8_t, 2048> buffer{};
        while (running_) {
            sockaddr_in sender{};
#ifdef _WIN32
            int sender_size = sizeof(sender);
#else
            socklen_t sender_size = sizeof(sender);
#endif
            const auto received = recvfrom(socket_, reinterpret_cast<char *>(buffer.data()),
                                           static_cast<int>(buffer.size()), 0,
                                           reinterpret_cast<sockaddr *>(&sender), &sender_size);
            if (received <= 0) {
                announce_to_connection(sequence++);  // receive timeout: keep announcing
                continue;
            }
            {
                std::lock_guard lock(mutex_);
                datagrams_.emplace_back(buffer.begin(), buffer.begin() + received);
            }
            if (silent_) {
                continue;
            }
            const auto reply = vehicle_heartbeat_frame(sequence++);
            static_cast<void>(
                sendto(socket_, reinterpret_cast<const char *>(reply.data()), static_cast<int>(reply.size()), 0,
                       reinterpret_cast<const sockaddr *>(&sender), sender_size));
        }
    }

    void announce_to_connection(std::uint8_t sequence) {
        if (silent_) {
            return;
        }
        const auto frame = vehicle_heartbeat_frame(sequence);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(connection_port_);
        static_cast<void>(sendto(socket_, reinterpret_cast<const char *>(frame.data()),
                                 static_cast<int>(frame.size()), 0,
                                 reinterpret_cast<const sockaddr *>(&address), sizeof(address)));
    }

    std::uint16_t connection_port_{};
    std::uint16_t port_{};
    Socket socket_{kInvalidSocket};
    std::atomic<bool> silent_{false};
    std::atomic<bool> running_{true};
    std::thread thread_;
    mutable std::mutex mutex_;
    std::vector<std::vector<std::uint8_t>> datagrams_;
};

// SET_POSITION_TARGET_LOCAL_NED as generated by mavgen for the pinned
// dialect; the wire id is stable across dialect revisions.
constexpr std::uint32_t kSetPositionTargetId = 84;

// One velocity setpoint decoded from the wire: SET_POSITION_TARGET_LOCAL_NED.
struct WireSetpoint {
    float vx{};
    float vy{};
    float vz{};
    float yaw_rate{};
    std::uint16_t type_mask{};
    std::uint8_t target_system{};
    std::uint8_t coordinate_frame{};

    bool is_zero() const {
        return vx == 0.0F && vy == 0.0F && vz == 0.0F && yaw_rate == 0.0F;
    }
};

WireSetpoint parse_setpoint(const nomad::mavlink::Message &message) {
    // Payload layout of the pinned dialect (generated headers): the velocity
    // floats start after time_boot_ms and the position triple, yaw_rate sits
    // after yaw, and the mask/target/frame bytes close the 53-byte payload.
    const auto &payload = message.payload;
    CHECK(payload.size() >= 53);
    WireSetpoint setpoint{};
    std::memcpy(&setpoint.vx, payload.data() + 16, sizeof(float));
    std::memcpy(&setpoint.vy, payload.data() + 20, sizeof(float));
    std::memcpy(&setpoint.vz, payload.data() + 24, sizeof(float));
    std::memcpy(&setpoint.yaw_rate, payload.data() + 44, sizeof(float));
    std::memcpy(&setpoint.type_mask, payload.data() + 48, sizeof(std::uint16_t));
    setpoint.target_system = payload[50];
    setpoint.coordinate_frame = payload[52];
    return setpoint;
}

std::vector<WireSetpoint> wire_setpoints(const std::vector<std::vector<std::uint8_t>> &datagrams) {
    std::vector<WireSetpoint> setpoints;
    for (const auto &datagram : datagrams) {
        std::size_t offset = 0;
        const std::span<const std::uint8_t> bytes(datagram);
        while (offset < bytes.size()) {
            std::size_t consumed = 0;
            const auto message = nomad::mavlink::decode_datagram(bytes.subspan(offset), consumed);
            if (!message.has_value()) {
                break;
            }
            offset += consumed;
            // SET_POSITION_TARGET_LOCAL_NED: the only message the core uses
            // for velocity setpoints.
            if (message->message_id == kSetPositionTargetId && message->system_id == 255 &&
                message->component_id == 190) {
                setpoints.push_back(parse_setpoint(*message));
            }
        }
    }
    return setpoints;
}

// Bounded wait for an authoritative condition; a timeout is a test failure
// with a diagnostic naming the missing observation.
template <typename Predicate>
void wait_until(Predicate ready, std::chrono::milliseconds budget, const char *what) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
        if (ready()) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error(std::string("timed out waiting for ") + what);
}

// Wait until the stand-in has recorded a non-zero setpoint followed (in wire
// order) by an all-zero setpoint, and return that zero setpoint.
WireSetpoint wait_for_zero_after_stream(VehicleStandIn &stand_in) {
    WireSetpoint zero{};
    wait_until(
        [&] {
            const auto setpoints = wire_setpoints(stand_in.recorded());
            for (std::size_t index = 1; index < setpoints.size(); ++index) {
                if (setpoints[index].is_zero() && !setpoints[index - 1].is_zero()) {
                    zero = setpoints[index];
                    return true;
                }
            }
            return false;
        },
        std::chrono::seconds(3), "a zero setpoint delivered after the streamed non-zero setpoints");
    return zero;
}

void require_safety_zero_fields(const WireSetpoint &zero) {
    CHECK(zero.type_mask == 0x07c7);  // ignore position/accel/yaw, command velocity + yaw rate
    CHECK(zero.coordinate_frame == 9);  // MAV_FRAME_BODY_OFFSET_NED
    CHECK(zero.target_system == 1);
    CHECK(zero.is_zero());
}

// Latch the vehicle and start an active velocity stream through the core.
std::unique_ptr<nomad::vehicle::Vehicle> start_streaming(nomad::mavlink::UdpMavlinkConnection &connection,
                                                         VehicleStandIn &stand_in) {
    CHECK(connection.wait_for_heartbeat(std::chrono::seconds(2)).has_value());
    nomad::safety::WatchdogPolicy policy{};
    policy.command_timeout = std::chrono::seconds(5);  // the watchdog must not stop this stream
    auto vehicle = std::make_unique<nomad::vehicle::Vehicle>(connection, policy);
    CHECK(vehicle->update_vio(true, 1.0F).success);
    CHECK(vehicle->set_velocity({0.5F, 0.0F, 0.0F, 0.0F}).success);
    wait_until(
        [&] {
            const auto setpoints = wire_setpoints(stand_in.recorded());
            return !setpoints.empty() && !setpoints.front().is_zero();
        },
        std::chrono::seconds(2), "the non-zero setpoint to reach the wire");
    return vehicle;
}

// The watchdog must put an all-zero setpoint on the wire when the command
// stream times out, after the non-zero setpoints it was streaming.
void test_watchdog_stop_delivers_zero_setpoint_on_the_wire() {
    std::fputs("[zero-delivery] watchdog stop\n", stderr);
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udpin:0.0.0.0:" + std::to_string(port));
    CHECK(connection.connect());
    VehicleStandIn stand_in(port);

    CHECK(connection.wait_for_heartbeat(std::chrono::seconds(2)).has_value());
    nomad::safety::WatchdogPolicy policy{};
    policy.command_timeout = std::chrono::milliseconds(150);
    policy.poll_interval = std::chrono::milliseconds(5);
    nomad::vehicle::Vehicle vehicle(connection, policy);
    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(vehicle.set_velocity({0.8F, 0.0F, 0.0F, 0.0F}).success);

    const auto zero = wait_for_zero_after_stream(stand_in);
    require_safety_zero_fields(zero);
    CHECK(!vehicle.velocity_control_active());
    CHECK(vehicle.last_velocity_stop_reason() == nomad::safety::WatchdogReason::command_timeout);

    connection.disconnect();
}

// Both explicit stop paths must zero the wire: Vehicle::stop_velocity() for
// the caller, and UdpMavlinkConnection::disconnect() for shutdown, which
// must send the zero before it closes the socket.
void test_stop_velocity_and_disconnect_deliver_zero_on_the_wire() {
    std::fputs("[zero-delivery] stop + disconnect\n", stderr);
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udpin:0.0.0.0:" + std::to_string(port));
    CHECK(connection.connect());
    VehicleStandIn stand_in(port);

    auto vehicle = start_streaming(connection, stand_in);
    CHECK(vehicle->stop_velocity().success);
    const auto stop_zero = wait_for_zero_after_stream(stand_in);
    require_safety_zero_fields(stop_zero);
    CHECK(!vehicle->velocity_control_active());

    // Reactivate, then shut the transport down while the stream is live: the
    // disconnect path must zero before closing the socket.
    CHECK(vehicle->set_velocity({0.5F, 0.0F, 0.0F, 0.0F}).success);
    const auto setpoints_before_disconnect = wire_setpoints(stand_in.recorded()).size();
    connection.disconnect();
    CHECK(!connection.is_connected());
    // Wait for the zero AND for it to be the newest recording: a datagram
    // sent before the disconnect may be recorded by the stand-in after the
    // zero, so only "the last setpoint is zero" proves delivery order.
    wait_until(
        [&] {
            const auto setpoints = wire_setpoints(stand_in.recorded());
            return setpoints.size() > setpoints_before_disconnect && setpoints.back().is_zero();
        },
        std::chrono::seconds(2), "the transport's zero setpoint on disconnect");

    CHECK(wire_setpoints(stand_in.recorded()).back().is_zero());
}

// When the vehicle heartbeat dies while a setpoint stream is active, the
// watchdog must observe the stale link and deliver the zero (SR-LNK-01) —
// the core may not keep steering a vehicle it can no longer see.
void test_stale_heartbeat_delivers_zero_on_the_wire() {
    std::fputs("[zero-delivery] stale heartbeat\n", stderr);
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udpin:0.0.0.0:" + std::to_string(port));
    CHECK(connection.connect());
    VehicleStandIn stand_in(port);

    CHECK(connection.wait_for_heartbeat(std::chrono::seconds(2)).has_value());
    nomad::safety::WatchdogPolicy policy{};
    policy.command_timeout = std::chrono::seconds(60);  // only the link may stop this stream
    policy.poll_interval = std::chrono::milliseconds(20);
    nomad::vehicle::Vehicle vehicle(connection, policy);
    CHECK(vehicle.update_vio(true, 1.0F).success);
    CHECK(vehicle.set_velocity({0.5F, 0.0F, 0.0F, 0.0F}).success);

    stand_in.set_silent(true);  // the vehicle heartbeat dies; the socket stays open
    // Keep the VIO feed alive (an independent visual-odometry computer may
    // still be streaming) so the link gate, not the VIO gate, is what stops
    // the vehicle; the heartbeat freshness window is 3 s.
    WireSetpoint zero{};
    wait_until(
        [&] {
            static_cast<void>(vehicle.update_vio(true, 1.0F));
            const auto setpoints = wire_setpoints(stand_in.recorded());
            for (std::size_t index = 1; index < setpoints.size(); ++index) {
                if (setpoints[index].is_zero() && !setpoints[index - 1].is_zero()) {
                    zero = setpoints[index];
                    return true;
                }
            }
            return false;
        },
        std::chrono::seconds(6), "the link-loss zero setpoint on the wire");
    require_safety_zero_fields(zero);
    CHECK(vehicle.last_velocity_stop_reason() == nomad::safety::WatchdogReason::heartbeat_stale);

    connection.disconnect();
}

// Vehicle destruction while a stream is live must deliver the zero through
// the destructor's shutdown ordering, on a link that is still healthy.
void test_vehicle_destructor_delivers_zero_while_streaming() {
    std::fputs("[zero-delivery] destructor while streaming\n", stderr);
    const auto port = find_free_udp_port();
    nomad::mavlink::UdpMavlinkConnection connection("udpin:0.0.0.0:" + std::to_string(port));
    CHECK(connection.connect());
    VehicleStandIn stand_in(port);

    auto vehicle = start_streaming(connection, stand_in);
    const auto setpoints_before_destruction = wire_setpoints(stand_in.recorded()).size();
    vehicle.reset();  // destructor sends the zero before shutting down
    wait_until(
        [&] {
            const auto setpoints = wire_setpoints(stand_in.recorded());
            return setpoints.size() > setpoints_before_destruction && setpoints.back().is_zero();
        },
        std::chrono::seconds(2), "the destructor's zero setpoint");

    CHECK(wire_setpoints(stand_in.recorded()).back().is_zero());
    connection.disconnect();
}

} // namespace

#ifdef _WIN32
struct WinsockGuard {
    WinsockGuard() {
        WSADATA data{};
        WSAStartup(MAKEWORD(2, 2), &data);
    }
    ~WinsockGuard() { WSACleanup(); }
};
#endif

int main() {
#ifdef _WIN32
    WinsockGuard winsock;
#endif
    try {
        test_watchdog_stop_delivers_zero_setpoint_on_the_wire();
        test_stop_velocity_and_disconnect_deliver_zero_on_the_wire();
        test_stale_heartbeat_delivers_zero_on_the_wire();
        test_vehicle_destructor_delivers_zero_while_streaming();
        std::fputs("[zero-delivery] all checks passed\n", stderr);
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
    return 0;
}
