# Development

The current tree builds a C++20 library and CLI with optional ROS 2 and Mission
Planner adapters. Edge Core source is deleted; some task/CI/deployment references
still need G1 repair. Source status and evidence belong in [migration](migration.md).

## Prerequisites and verified local checks

Use Git, Pixi, CMake and a C++20 compiler. The current codec build also requires
the pinned ArduPilot MAVLink submodule and Python/mavgen. This is a build-time
dependency; C++ runtime use does not require Python, ROS or a GPU.

~~~sh
pixi run test-core
pixi run test-python
pixi run lint
pixi run format-check
pixi run complexity-check
pixi run docs-build
~~~

test-core configures/builds before CTest; build-core builds without tests.
format-check is read-only with respect to source; format rewrites source and is
not appropriate for a documentation-only review of unrelated migration work.
docs-build is the strict ProperDocs site check.

Do not use dev, dev-build, test-api or the old test coverage task as current
quickstarts: they reference deleted Edge Core files. Compose and CI still carry
deleted-image references. dev-up/sitl startup must be repaired and verified at
G1; a task name existing in pixi.toml is not evidence that it works.

## Test layers

| Layer | Current checks | Required expansion |
|---|---|---|
| C++ | Nine CTest targets: core, codec, safety, output, UDP, zero, VIO, limits, fence | Authority, per-field freshness, cancellation, MAVSDK and vehicle-class coverage |
| Python | pytest includes client contracts, traceability, harnesses, profiles and video tools | Mock competition server/traffic, perception replay and tracker fixtures |
| ROS | ros2/nomad_ros translation plus tests/ros integration | Bounded callbacks, acquisition-time/frame validation, command-owner integration |
| Mission Planner | lint-plugin and test-plugin-* helper scripts | Ownership, capabilities, stale displays, action lifecycle and replay |
| SITL | core-sitl-* and sitl-fence | Current artifacts, QuadPlane transitions, competition scenarios and independent faults |
| Hardware | No evidence collected in this review | Selected board/sensor/radio/payload/endurance and task gates |

Tests/ros and live SITL checks are environment-gated; report skips explicitly.
The checked cpp_traceability block only proves references exist. It does not
prove every safety requirement is covered or satisfied.

## SITL discipline

Use an isolated identified simulator with known state. Serialize scenarios that
share vehicle state. Every important test establishes state, performs one action,
observes authoritative outcome, asserts independent conditions and restores
state when required. A relay ACK is not physical sample collection, and LAND
mode is not a completed landing.

Existing tasks: core-sitl-status, core-sitl-command-flow, core-sitl-mission,
core-sitl-velocity-watchdog, core-sitl-geofence, core-sitl-payload,
core-sitl-link-loss, core-sitl-link-recovery, core-sitl-zero-delivery,
core-sitl-gcs-heartbeat and sitl-fence. Once G1 startup is repaired, use them
against the configured isolated endpoint with no hardware path attached.

Add a pinned ArduPlane/QuadPlane SITL path for Task 1. Copter mode numbers and
velocity-stop behavior cannot stand in for transition, cruise and VTOL landing
tests. Gazebo/Isaac are optional sensor-evidence tools; they are not prerequisites
for basic unit or server-contract tests.

## Adapter and optional build checks

MAVSDK build-core-mavsdk and mavsdk-phase-a-smoke are opt-in Phase A tasks; the
latter requires live SITL. They do not switch production to MAVSDK. Follow
[MAVSDK parity gates](mavsdk-adoption.md).

ROS builds use the separate ament package and supported image; test-ros-integration
runs its real adapter tests. Current source still has blocking callbacks; passing
existing tests does not prove the target callback deadline contract.

Mission Planner uses Windows/.NET Framework 4.8 and its reference assemblies.
Use lint-plugin and relevant test-plugin-* tasks for non-deploying checks.
build-plugin invokes a script that can install/overwrite the local plugin;
inspect it and obtain deployment authorization before running it. Do not confuse
a pure helper test with full plugin integration.

## Contribution and evidence workflow

1. Read AGENTS and deeper guidance; trace owning symbols, callers and tests.
2. Define a concrete falsification test and keep one active work item.
3. Make one focused change; preserve unrelated staged/unstaged work.
4. Run focused tests, then full relevant checks; report every skip/failure.
5. Review the final diff and record requirement, artifact/config identity,
   independent observations, measured values, thresholds and limitations.
6. Update the canonical subject owner; avoid copying status into several plans.

Use a focused branch for implementation and the repository commit prefix style.
Do not stage, commit, push or deploy without an explicit request. MAVSDK
implementation needs a focused merge request with unit and integration evidence;
this documentation review only plans that work.

Size/complexity rules and technical-debt format remain in AGENTS. Keep C++ public
headers in include/nomad, implementation in src, and ROS/Python/vendor types out
of the core API. Retained Python is for tools, perception and tests, never a
parallel vehicle state machine.
