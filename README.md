# NOMAD

NOMAD is a C++20 vehicle-control core with CLI, Mission Planner and ROS 2 clients,
prepared for the AEAC SUAS 2027 Wildlife Monitoring preview. ArduPilot owns
stabilization, EKF, navigation execution and failsafes.

## Current status

The working migration tree contains a tested C++ core, UDP MAVLink implementation,
basic Copter operations, safety/watchdog/fence/payload primitives and adapters.
Edge Core source has been removed. Build and profile/service wiring repairs are
recorded in the migration gates; older setup/provisioning paths still need repair.
MAVSDK is an early competition prerequisite but currently only has an optional
smoke target. QuadPlane, competition telemetry/traffic and task workflows remain
implementation work.

Task 1 targets a lightweight VTOL with ground GPU vision. Task 2 targets a quad
below 15 kg with optional onboard compute. All three product profiles remain:
onboard_companion, groundstation_gpu and groundstation_minimal. See the canonical
documents for confirmed directions, provisional requirements and unresolved choices.

## Start here

- [Delivery plan](PLAN.md) and [working ledger](TODO.md)
- [Requirements and user decisions](docs/prd.md)
- [Architecture](docs/architecture.md)
- [Migration status and evidence gates](docs/migration.md)
- [Development](docs/development.md), [operations](docs/operations.md)
  and [safety case](docs/safety.md)

~~~sh
pixi run test-core
pixi run test-python
pixi run docs-build
~~~

These checks do not start hardware. Core and daemon-free configuration checks are
current; live Docker/SITL/ROS startup remains an open G1 qualification gate. Real
configuration stays in ignored local storage.

## Layout

C++ headers and implementation live in include/nomad and src; tests in tests.
ros2 contains adapters, python contains retained tools/perception work,
mission_planner contains the client, and config/docker/infra contain deployment
support. Public core headers do not depend on clients, ROS, Python or GPU SDKs.

## License

Apache 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
