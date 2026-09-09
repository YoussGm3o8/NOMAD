# Migration and release gates

Source review baseline: 2026-09-08, including the existing staged, unstaged and
untracked migration changes. This document owns implementation status, the
cutover inventory and gate evidence. [PRD](prd.md) owns requirements and decisions;
[architecture](architecture.md) owns the target; TODO is the working ledger.

## Current implementation inventory

"Implemented" below means source and focused tests exist, not flight readiness.

| Area | Source and tests inspected | Actual scope and remaining limitation |
|---|---|---|
| C++ foundation | CMakeLists.txt; include/nomad; src; nine CTest targets | Library and CLI build; Python/mavgen build dependency, no Python runtime dependency |
| MAVLink | src/mavlink; core_test, codec_golden_test, udp_connection_test | Generated dialect, CRC/framing, UDP, ACKs, typed telemetry, heartbeat/relay handling; native serial/TCP absent |
| Vehicle | src/vehicle/vehicle.cpp; core_test.cpp | Arm/mode/takeoff/goto/land/RTL and state checks; Copter modes hardcoded; LAND/RTL success verifies mode, not task completion |
| Missions | src/mission/executor.cpp; core_test.cpp | Synchronous small step executor; no integrated cancellation, persisted resume, survey or Task 2 workflow |
| Safety | src/safety; safety_test, fence_config_test, velocity_config_test, vio_source_test | Finite/range gates, VIO-conditioned velocity, watchdog, configured target fence, upload/readback and payload interlock |
| Stop delivery | tests/zero_delivery_test.cpp; scripts/dev/core_sitl_zero_delivery.py | Live loopback wire tests exist; whole-link outage cannot guarantee delivery; current live SITL result still required |
| Mission Planner | NomadCoreClient, OutputController, FlightModeController, GimbalController, BoundaryManager, MPFenceUploader | goto/discrete outputs use CLI; direct parameter/mode/gimbal/fence paths and UI-owned decisions remain |
| ROS 2 | ros2/nomad_ros/src/node.cpp, translation.cpp; tests/ros | Owns a Vehicle, telemetry topics, VIO health/source gate and Trigger services; blocking callbacks, no selected estimator or navigation fusion |
| Video | python/tools/simple_video_bridge.py, video_bridge_server.py; test_simple_video_bridge.py | ROS image to GStreamer/RTSP and media-control HTTP; no validated capture/CV/VIO product pipeline |
| Profiles | scripts/profile.py; three new profile env files; test_deployment_profiles.py | Names/values/config sync tested; no runtime capability enforcement or hardware workload qualification |
| MAVSDK | CMake opt-in target; examples/mavsdk_phase_a_smoke.cpp; runner and CI wiring | Phase A scaffolding exists; production still uses current codec; command/flight parity absent |
| Competition | No dedicated implementation found in src/include/ROS/Python/plugin scans | Web telemetry/events, traffic model/deconfliction, herd survey, tracker/path and sampling workflows are open |

The working tree removes the Edge Core source/service/API and many camera/
SLAM/terminal UI components. Their removal is already present, not performed by
this documentation change. Generic payload panels, RTSP playback, ROS adapter
and Python video tool remain. Do not restore deleted architecture to compensate
for missing workflows.

## Contradictions and concrete follow-up

| ID | Finding | Required resolution |
|---|---|---|
| C01 | PLAN/TODO previously repeated obsolete phases and incompatible pass counts | This document owns evidence; PRD decisions and TODO tasks link here |
| C02 | pixi dev/test-api/dev-build, Compose edge_core and test.yml dev-image reference deleted files | Repair tasks/CI/Compose in G1; do not advertise dev/dev-up/sitl as verified quickstarts yet |
| C03 | Older docs said Edge Core still runs and only two deployment profiles exist | Three profiles are product scope; qualify actual entrypoints in G3 |
| C04 | CLI key check accepts any nonempty value; CLI logs admission before outcome; library has no inherited auth/audit | OS trust boundary now; authenticated client protocol and full lifecycle audit at G2 |
| C05 | Separate CLI invocations, ROS Vehicle and direct plugin writers can conflict | One integrated command owner, explicit manual handover and per-session cancellation |
| C06 | FlightModeController.EmergencyLand changes descent parameters using old CONOPS wording | Do not carry that rule into 2027; review/remove behavior in a later code change and test safe vehicle-specific abort |
| C07 | Generic servo/relay paths bypass release_payload's consuming interlock; plugin has its own timers/confirmation | Reserve hazardous channels, unify authorization in core, provide independent output timeout/feedback |
| C08 | ROS callbacks wait synchronously; receipt timestamps and mixed odometry frames can misrepresent freshness/frame | Bounded worker operations; acquisition-time/frame contract and independent axis/time tests |
| C09 | VehicleState has validity flags but no per-field age; connection freshness is not position freshness | Add independent field timestamps; reject fresh-heartbeat/stale-position decisions |
| C10 | groundstation profile flags imply capabilities; VIO_SOURCE_REQUIRED does not select core policy | Validate effective config and runtime readiness; minimal must not fabricate VIO |
| C11 | Profile sync still writes retired Jetson keys; onboard endpoint lacks CLI scheme; ground profiles may retain old core endpoint | Normalize/validate endpoint and complete profile switches; prove no stale settings survive |
| C12 | Health/time autostart flags outlive removed services; media HTTP binds broadly without auth | Inventory effective service wiring, replace necessary monitoring/time support, restrict/authenticate media control |
| C13 | Copter GUIDED=4, LAND=9, RTL=6 and demo flows are treated as board-generic support | Detect/validate vehicle class; qualify ArduPlane/QuadPlane and Copter separately before Task 1 |
| C14 | Prior ZED/sim components were deleted; current sensor/workload path is unspecified | Preserve optional compute goals, choose supported capture/provider paths; no ZED prerequisite |
| C15 | Scan-based failsafe and traceability tests do not prove all command surfaces are safe | Semantic parameter/output authorization and actual fault tests; retain scans as structural checks |
| C16 | MAVSDK adoption was both "decided" and awaiting approval; Phase A was marked complete/in-progress in several places | User confirmed early release prerequisite; one transport record and G-M gates below |

Telemetry frame review must include the ROS odometry NED label with up-positive
position and body-frame metadata, and velocity sign conversion through both
adapter and wire encoder. Do not declare an axis correct from a test mirroring
the same conversion.

## Evidence contract

For each gate retain a sanitized manifest: requirement IDs, exact repository
artifact/diff identity, firmware and dependency pins, profile/config hash, test
command, environment class, dates, independent witness, measured values versus
approved thresholds, pass/fail/skips, artifact reference and accountable reviewer.
Store recordings/logs privately where appropriate; never commit raw transcripts,
secrets, machine-specific endpoints, wildlife coordinates or build output.

A mock proves the adapter contract against the mock; official-server acceptance
is separate. A unit test proves its scenario; SITL does not prove electrical
de-energization, RF independence or aerodynamic feasibility. A configured CI job
is not a passed run. A skipped required scenario leaves its gate open.

Historical notes (2026-09-03–06) reported Copter 4.7.0/4.7.1 SITL success for
status, command flow, mission, velocity watchdog, fence upload/readback, payload,
link loss/recovery and Linux GCS-heartbeat with negative control. These reports
have not been independently re-established on this working tree. Historical
MAVSDK build/footprint measurements remain in its decision record.

Historical live containment and first Linux zero-delivery runs were explicitly
open despite stronger wording elsewhere. Preserve them as open until artifacts
resolve the discrepancy. No hardware evidence was supplied for this review;
hardware availability beyond the user's stated selections is not inferred.

## Gate sequence and accountable roles

Roles below need named owners (D10). Dependencies order implementation, not
permission to operate hardware. No gate is closed merely by this planning change.

### G0 — Planning baseline (lead + competition lead)

Reconcile these docs, preserve stable safety IDs, resolve links and traceability.
Read the full CONOPS when available and attach source sections to P requirements.
Record user decisions and keep remaining hardware/rule thresholds TBD.
Exit: strict docs build and traceability test pass; no unsupported requirement
claims; assigned decision owners and CONOPS reconciliation remain necessary.

### G1 — Executable baseline (build/integration lead; depends on G0 planning)

Repair C02/C10–C12 deployment/task leftovers without reintroducing Edge Core.
Retain video and both optional compute placements. Demonstrate fresh checkout
builds and selected simulation startup with no deleted-path dependencies.
Exit: test-core, test-python, lint, format-check, complexity-check, docs-build,
plugin compile/helper checks and supported ROS build/tests. Check command paths
against actual files; current runtime repairs require their own focused change.

### G-M — MAVSDK adoption (transport lead; early prerequisite after G1)

Complete Phase A build/dependency/license/SITL evidence, then command, velocity,
watchdog/zero, GCS heartbeat and fence parity. Preserve unit fault coverage,
independent wire observations and CLI/plugin compatibility. Test both Copter
and the chosen QuadPlane firmware as support is introduced; existing Copter
tests alone cannot qualify Task 1.

Create focused implementation changes and a merge request with unit tests,
integration evidence, requirement mapping and limitations. Upstream ArduPilot
fixes need reproducible tests and tracked merge requests/PRs; publication follows
the contribution authorization workflow. This planning pass does not implement
or publish those changes.

Exit: phases A–E in [MAVSDK adoption](mavsdk-adoption.md) pass; default production
runtime demonstrably uses MAVSDK; legacy deletion passes gates below; transitive
notices and supported firmware matrix are recorded. Phase F upstream acceptance
may lag, but required patches must be maintained and reviewed. G-M is mandatory
for G8 and precedes dependent integrated competition command work; isolated
server/CV prototypes may proceed without waiting.

### G2 — One authority and aircraft semantics (core + safety leads; G1/G-M)

Add the small persistent runtime and client boundary; unify mission/payload/
traffic state, stop/cancel/restart semantics and audit. Correct C04–C09/C13/C15.
Implement vehicle-class capability checks before interpreting mode numbers.
Plan Task 1 around ArduPlane/QuadPlane mission execution and Task 2 around Copter;
prefer autopilot execution of reviewed survey missions, with C++ planning,
validation, supervision and outcome ownership. Mission upload/progress/abort is
new work; current synchronous demo is not a survey implementation.

Exit: two conflicting clients cannot both command; manual takeover inhibits
automatic resume; replay/restart cannot refire payload; stale position with
fresh heartbeat is refused; blocked service cannot starve safety deadlines.
Unit and SITL tests verify class mismatch rejection, Copter behavior, QuadPlane
takeoff/transition/cruise/return/VTOL landing and aborted/failed transition.
Use vehicle-appropriate holds: fixed-wing flight cannot be stopped by zero
velocity. Critical operation completion requires authoritative state.

### G3 — Profile qualification (integration + perception leads; G2)

For each profile test clean boot, core/ROS/video process absence, missing GPU/
camera, wrong endpoint, restart, link loss and capability transitions.
Task 1 primary: groundstation_gpu with aircraft FPV capture and optional Pi Zero
LTE bridge. Task 2 candidate: onboard_companion if useful autonomy is required;
Jetson installation is not decided. groundstation_minimal remains supported.

Exit: minimal C++ telemetry/eligible mission/output safety works without ROS/
camera/GPU; both compute profiles run selected workloads with measured CPU/GPU,
thermal, bandwidth, frame age and end-to-end latency under load. Raw/synchronized
VIO feeds are separately qualified if needed. Profile strings are insufficient.

### G4 — Server and traffic (integration + core leads; G2, D07/D08)

Implement schema adapter and deterministic mock; sample/send at 1 Hz plus events,
consume/process 1 Hz traffic. Define data age, retry, queue, clock and outage
policies with approved thresholds. Advisory-first is the user's initial scope;
automatic maneuver decisions remain TBD after CONOPS.

Exit: record send and independent receipt timestamps, missed cycles and jitter;
events reconcile across restart without duplicate effect; malformed/auth-expired/
rate-limited/offline server cannot block command handling. Inject crossing,
head-on, overtaking, diverging, stationary, duplicate-ID and stale traffic with
ground truth; measure false/missed advisories and warning time against approved
thresholds. Missing feed shows unknown. Official-server integration and required
simulated-UAV cooperation are separate acceptance runs, not mock-only closure.

### G5 — Task 1 mission (mission + perception + flight leads; G2–G4)

Cover area planning, CV evidence, count/identity review, simulated cooperation,
traffic advisories and safe return/landing on the VTOL. Characterize video
capture access from Walksnail equipment before assuming frame availability.
GNSS/Here4 RTK is the intended navigation baseline; VIO is not a Task 1 dependency.

Exit: independent known animal/marker counts and identities, held-out imagery,
coverage and geolocation errors, duplicate/occlusion cases and review audit.
QuadPlane SITL plus authorized field rehearsal verifies complete task with
single-battery reserve, transitions and landing. Required accuracy/distance/
duration thresholds come from CONOPS and D08, not this plan.

### G6 — Task 2 payload mission (payload + mission + safety leads; G2–G4, D04)

Custom tracker technology (possibly ESP32), attachment and sample mechanism remain
undecided. Implement tracker identity/position ingestion, association and mapped
path with uncertainty/gaps. Explicit action authorization is initial scope;
autonomous sampling remains a later CONOPS decision.

Exit: known ground-truth trajectory and measured error/age; wrong tracker,
packet loss, replay, reboot, RF interference and depleted tracker battery cases.
Observe tag attachment and simulated sample collection with independent feedback,
measure quantity/containment, and test jam/no-contact/relay-off failure/abort.
Task 2 battery swap proves disarm, safe payload, persistent mission records,
expired permissions and deliberate resume without duplicate tagging/sampling.

### G7 — Hardware and operational safety (hardware + flight/safety leads)

Prerequisites: relevant software and SITL gates, selected hardware and separately
authorized test procedure. Cube Orange and a custom ArduPilot controller need
separate pin/power/UART/PWM/timing verification even with the same protocol.

Exit: calibrated all-up mass below 15 kg for each configuration (the "15 kg class"
quad must still be below the limit); measured CG, power, thermal and endurance
reserve including payload, compute, radios and transition energy. Verify Here4/
RTK base correction path, fix degradation and navigation fallback without
assuming RTK availability or accuracy from the product name.

Bench evidence covers props-safe setup, sensor calibration/time, camera access,
actual output de-energization on link/power/process faults, independent manual
control and common-mode link failures. A disconnected link cannot deliver stop;
verify ArduPilot behavior independently. Authorized field tests cover aircraft-
specific abort, transition failure, landing, traffic procedures and safe payload.
Re-read safe configuration and disarmed state after each session; record failures.

### G8 — Competition release (release + safety + competition leads; all gates)

Exit: MAVSDK is the tested production transport; both task rehearsals run on
versioned artifacts/configurations; required profiles and firmware combinations
pass their suites; official server contract is tested; no unresolved safety-
critical defect or acceptance threshold. Optional unavailable features are
explicit and do not conceal a failed required capability.

Package core, plugin, selected adapters, dependency notices, configuration
templates and operator procedures reproducibly. Test install/upgrade/rollback
and startup on clean qualified hosts; rollback to a prior qualified build never
auto-resumes motion. Exercise expired credentials, replay, disk-full, process
crash and server/video overload with complete audit. Review proposal/demo/report
evidence, assign operator roles and obtain separate operational authorization.

## Deletion and cutover rules

Before any further removal: inventory callers, assign replacement or explicit
feature unavailability, retain requirement/fault tests, record SITL/hardware
evidence appropriate to the behavior, pass each required profile path, and
remap safety traceability. Do not delete optional compute because its former
control UI was REST-based. Retire aliases/settings only after verified migration.

The two codecs may coexist temporarily for comparison; only one owns commands
in a test/deployment. Keep golden wire references until equivalent semantic/
wire evidence survives the switch. Do not interpret this plan as permission to
delete the current codec before G-M.

## Checks run for this documentation review

- 2026-09-08: pixi run test-core — 9/9 CTest targets passed (Windows Debug).
- 2026-09-08: pixi run test-python — 251 passed, 3 skipped.
- 2026-09-08: strict docs build, Ruff lint/format-check, complexity-check and
  non-deploying plugin lint/compile passed. Size checks retain existing baseline
  debt; a passing policy check is not a claim that every file meets target size.
- 2026-09-08: safety traceability passed after documentation changes; plugin
  core-client, payload-interlock and dual-link tests passed.
- 2026-09-08: all 72 checked local Markdown file links resolved; git diff
  whitespace check passed. Existing staged content and non-document files were
  compared against the pre-edit snapshot to verify preservation.
- No SITL startup, hardware actuation, deployment, hosted CI or MAVSDK rebuild
  was performed in this review. Earlier pass counts are historical only.
