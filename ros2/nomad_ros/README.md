# nomad_ros

ROS 2 adapter for the NOMAD C++ core. It is a client, not a second vehicle
implementation: it owns a UDP MAVLink connection and one core `Vehicle`, then
translates standard ROS messages into core API calls. ArduPilot-facing safety,
command validation, and the velocity watchdog stay in `nomad_core` (see
`docs/architecture.md`, `docs/migration.md`, and `docs/safety.md`).

Current standalone adapter behavior is described below. The integrated G2
target uses one shared command owner instead of running this Vehicle alongside
plugin-spawned CLI writers. Current callbacks block on vehicle/telemetry waits;
bounded callback execution is open work, not an implemented guarantee.

## Build

The adapter links `nomad_core` from the repository root as a subproject, so it
builds anywhere the repo is mounted into a ROS 2 workspace:

```bash
mkdir -p /ws/src && cp -r . /ws/src/nomad   # repo root must be /ws/src/nomad
cd /ws
colcon build --packages-select nomad_ros
source install/setup.bash
```

The repo's `nomad-sim-ros` Docker image contains the Humble toolchain
(`colcon`, `rclcpp`, `ament_cmake`) for this build. It does not require a
camera or perception provider.

## Run

```bash
ros2 launch nomad_ros nomad_vehicle.launch.py
```

Parameters are declared in `config/params.yaml` and loaded by the launch file:
`endpoint` (MAVLink UDP), publish rate, the reviewed velocity envelope,
minimum VIO confidence, and the watchdog/VIO freshness timeouts that configure
the core `WatchdogPolicy`.

## Contract

Publishers (telemetry from the core `VehicleState`):

- `/nomad/fix` — `sensor_msgs/NavSatFix`, WGS-84 position + MSL altitude.
- `/nomad/battery` — `sensor_msgs/BatteryState`.
- `/nomad/odom` — `nav_msgs/Odometry`; frame `ned` at the vehicle:
  `pose.position.z` is relative altitude (up positive), `twist.linear` is
  north/east/down m/s, orientation is the FC roll/pitch/yaw.
- `/nomad/connected` — `std_msgs/Bool`.

Subscriptions:

- `/nomad/cmd_vel` — `geometry_msgs/TwistStamped`, vehicle FLU convention
  (x forward, y left, z up, yaw CCW positive). Converted to the core's FRD
  velocity command; non-finite commands are rejected. The core clamps the
  setpoint to its reviewed limits and requires armed + GUIDED + a fresh,
  healthy VIO feed; its watchdog stops the setpoint when input goes stale.
- `/nomad/vio_health` — `std_msgs/Bool`.
- `/nomad/vio_confidence` — `std_msgs/Float32`.
- `/nomad/vio_source` — `std_msgs/String`; must match the configured source
  identity. No provider is selected by default; health, confidence, and source
  are combined into one fresh sample before the core receives a command.

Services (thin wrappers over core `Vehicle` calls):

- `/nomad/arm`, `/nomad/disarm`, `/nomad/land`, `/nomad/rtl` —
  `std_srvs/srv/Trigger`.

## Frame and VIO notes

The current odometry mixes an NED label with up-positive position and body-frame
metadata; do not use it as a navigation/fusion input until G2 corrects and tests
the frame contract. Validate velocity signs independently through both adapter
and wire conversion. A future perception adapter must
publish health, confidence, and source identity. The ROS node validates all
three as one fresh sample with `VioSourceValidator` before the core receives
it; a missing, stale, unhealthy, low-confidence, or mismatched sample cannot
open the velocity path. No VIO provider is enabled by the default parameters.
The configured limits are loaded from `max_velocity_xy`, `max_velocity_z`, and
`max_yaw_rate`; malformed values fail closed. If the vehicle disarms, leaves
GUIDED, loses heartbeat, or VIO goes stale, the core stops velocity on its own
— the adapter does not re-implement those decisions.
