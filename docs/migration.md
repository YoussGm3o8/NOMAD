# Migration and release gates

Source review baseline: `fab9f46`, inspected 2026-09-10 with a clean working tree;
includes planning `d31b0aa`, runtime `3ef38e7`, profiles `51f309e`, setup `a9762b0`
and MAVSDK Phase A `6922371`. Earlier evidence below retains its original dates.
This document owns implementation status, the
cutover inventory and gate evidence. [PRD](prd.md) owns requirements and decisions;
[architecture](architecture.md) owns the target; TODO is the working ledger.

## Current implementation inventory

"Implemented" below means source and focused tests exist, not flight readiness.

| Area | Source and tests inspected | Actual scope and remaining limitation |
|---|---|---|
| C++ foundation | CMakeLists.txt; include/nomad; src; ten CTest targets | Library and CLI build; Python/mavgen build dependency, no Python runtime dependency |
| MAVLink | src/mavlink; core_test, codec_golden_test, udp_connection_test | Generated dialect, CRC/framing, UDP, ACKs, typed telemetry, heartbeat/relay handling; native serial/TCP absent |
| Vehicle | src/vehicle/vehicle.cpp; core_test.cpp | Arm/mode/takeoff/goto/land/RTL and state checks; Copter modes hardcoded; LAND/RTL success verifies mode, not task completion |
| Missions | src/mission/executor.cpp; core_test.cpp | Synchronous small step executor; no integrated cancellation, persisted resume, survey or Task 2 workflow |
| Safety | src/safety; safety_test, fence_config_test, velocity_config_test, vio_source_test | Finite/range gates, VIO-conditioned velocity, watchdog, configured target fence, upload/readback and payload interlock |
| Stop delivery | tests/zero_delivery_test.cpp; scripts/dev/core_sitl_zero_delivery.py | Live loopback wire tests exist; whole-link outage cannot guarantee delivery; current live SITL result still required |
| Mission Planner | NomadCoreClient, OutputController, FlightModeController, GimbalController, BoundaryManager, MPFenceUploader | goto/discrete outputs use CLI; direct parameter/mode/gimbal/fence paths and UI-owned decisions remain |
| ROS 2 | ros2/nomad_ros/src/node.cpp, translation.cpp; tests/ros | Owns a Vehicle, telemetry topics, VIO health/source gate and Trigger services; blocking callbacks, no selected estimator or navigation fusion |
| Video | python/tools/simple_video_bridge.py, video_bridge_server.py; test_simple_video_bridge.py | ROS image to GStreamer/RTSP; control HTTP is loopback-only; no validated capture/CV/VIO product pipeline |
| Profiles | scripts/profile.py; three product profile files; test_deployment_profiles.py | Canonical endpoint and stale-setting checks exist; optional workloads and hardware remain unqualified |
| MAVSDK | CMake opt-in target; Phase B MAVSDK `MavlinkConnection` (commands, telemetry, deterministic peer fixture); qualified telemetry smoke; provenance and CI gates | Phase B transport parity passes its peer fixture (accepted/denied/timeout ACKs, COMMAND_LONG/COMMAND_INT, wrong identity, command parity for mode/takeoff/goto/land/RTL/servo/relay/gimbal-config/user-command, unlatched GCS-heartbeat announcement, coalesced-datagram handling, live-to-stale link observation, body-frame velocity parity with the zero setpoint on disconnect, a wire-confirmed data-stream request, and the core fence upload/readback/enable-verification of Phase D) and the CLI can select it with `--transport mavsdk`; mission parity and production cutover (Phase E) remain open, and production still uses the current codec |
| Competition | No dedicated implementation found in src/include/ROS/Python/plugin scans | Official telemetry, traffic model/deconfliction, herd survey, tracker/path and sampling workflows are open; events await contract |

The working tree removes the Edge Core source/service/API and many camera/
SLAM/terminal UI components. Their removal is already present, not performed by
this documentation change. Generic payload panels, RTSP playback, ROS adapter
and Python video tool remain. Do not restore deleted architecture to compensate
for missing workflows.

## Contradictions and concrete follow-up

| ID | Finding | Required resolution |
|---|---|---|
| C01 | PLAN/TODO previously repeated obsolete phases and incompatible pass counts | This document owns evidence; PRD decisions and TODO tasks link here |
| C02 | Deleted build/task/image references repaired in the current G1 slice; package now includes retained Python tools | Local checks recorded below; live image/SITL/ROS qualification remains open |
| C03 | Older docs said Edge Core still runs and only two deployment profiles exist | Three profiles are product scope; qualify actual entrypoints in G3 |
| C04 | CLI key check accepts any nonempty value; CLI logs admission before outcome; library has no inherited auth/audit | OS trust boundary now; authenticated client protocol and full lifecycle audit at G2 |
| C05 | Separate CLI invocations, ROS Vehicle and direct plugin writers can conflict | One integrated command owner, explicit manual handover and per-session cancellation |
| C06 | FlightModeController.EmergencyLand writes LAND_SPEED/WPNAV_SPEED_DN and returns mode dispatch; plugin comments claim termination compliance | v1.0 does require minimum 2 m/s rotary descent, but dispatch/parameters prove neither touchdown nor five-second/all-mode/C2-loss termination; redesign ownership and prove aircraft-specific behavior at G2/G7 |
| C07 | Generic servo/relay paths bypass release_payload's consuming interlock; plugin has its own timers/confirmation | Reserve hazardous channels, unify authorization in core, provide independent output timeout/feedback |
| C08 | ROS callbacks wait synchronously; receipt timestamps and mixed odometry frames can misrepresent freshness/frame | Bounded worker operations; acquisition-time/frame contract and independent axis/time tests |
| C09 | VehicleState had validity flags but no per-field age, so connection freshness was not position freshness. Position freshness is now enforced by the core: `Vehicle::wait_for_location`/`wait_for_altitude` reject a position sample older than the configurable `position_freshness_timeout`, and battery/GPS/attitude timestamps are stamped but not yet gated | Add independent field timestamps; reject fresh-heartbeat/stale-position decisions |
| C10 | Product profiles previously autostarted unqualified optional workloads; VIO_SOURCE_REQUIRED does not select core policy | Optional autostarts now remain off; G3 must qualify providers and runtime readiness before enabling them |
| C11 | Profile sync wrote retired Jetson keys and allowed ambiguous/stale endpoints | Product profiles now validate canonical endpoints and clear retired/stale profile-owned settings |
| C12 | Deleted service owners remained in CLI/systemd/config; media HTTP bound broadly without auth | Effective service inventory is reconciled and unauthenticated media control is restricted to loopback |
| C13 | Copter GUIDED=4, LAND=9, RTL=6 and demo flows are treated as board-generic support | Detect/validate vehicle class; qualify ArduPlane/QuadPlane and Copter separately before Task 1 |
| C14 | Prior ZED/sim components were deleted; current sensor/workload path is unspecified | Preserve optional compute goals, choose supported capture/provider paths; no ZED prerequisite |
| C15 | Scan-based failsafe and traceability tests do not prove all command surfaces are safe | Semantic parameter/output authorization and actual fault tests; retain scans as structural checks |
| C16 | MAVSDK adoption was both "decided" and awaiting approval; Phase A was marked complete/in-progress in several places | User confirmed early release prerequisite; one transport record and G-M gates below |
| C17 | Preview plan asserted Task 2 battery swaps and event uploads, left all task details TBD | Replaced with AE27 inventory; swaps and event protocol remain Q03/Q04, not official facts |
| C18 | Generic survey/animal identity plan omitted specific deer clusters, tag codes and timed text/CSV submissions | G5/G6 now use exact task artifacts and scoring oracles |
| C19 | Advisory-first wording could imply display alone meets deconfliction; payload autonomy was wholly pending CONOPS | Actual cylinder avoidance required; manual operation permitted; Task 2 autonomy separately scored, D05/Q06 still needed |
| C20 | Generic link-loss recovery and fence upload could be mistaken for competition termination compliance | Independent all-mode termination, C2-path loss, 100 m AGL and verified hard-polygon containment are G7 gates; internal soft inset remains unchanged per U-FEN-01 |
| C21 | Profile README still claimed template credentials/endpoint defects; SITL README retained removed task; CONTRIBUTING described existing Edge Core | Local summaries reconciled to repaired source; no runtime code changed |
| C22 | GeofenceConfig.MaxAltitudeAglMeters and NOMADBoundaryView default to 122 m; preset display checks 122 m; optional soft-from-hard inset uses a project buffer | CONOPS ceiling is 100 m AGL; qualify core-owned limits and truthful UI, reject old competition presets, preserve the plugin internal inset unchanged per U-FEN-01; no second official polygon required (GAP-05/G7) |
| C23 | `Vehicle::motor_test` sent MAVLink command ID 139, which is not a `MAV_CMD` entry in the pinned dialect (`MAV_CMD_DO_MOTOR_TEST` is 209), so no autopilot handler matched it; `tests/output_command_test.cpp` pinned 139 and no SITL scenario exercised the verb, so nothing caught it. Now sends 209, with the parameter layout confirmed on live SITL (`MAV_RESULT_ACCEPTED`), the unit test repinned, `tests/test_command_ids.py` resolving every hand-typed command id against the pinned dialect definition, and a motor-test parity case in the MAVSDK fixture | Keep the dialect-id gate green when a verb is added or the pinned submodule moves; the acceptance is an acknowledgement plus status text, so obtain an independent observation of motor behaviour before relying on it for a flight clearance |
| C24 | The `core-sitl-*` scenarios verify against a host-side SITL copy fed by MAVProxy, but ArduPilot streams position, attitude and extended status only after a GCS requests the group while the C++ core deliberately never requests streams (measured: a passive vehicle link delivered ~1 Hz heartbeats and no position/attitude until `REQUEST_DATA_STREAM`), so with the Python edge service removed nothing asked for the streams the copy is read for and it carried heartbeats only. The SITL stack now applies `docker/sitl-streams.parm` (SERIAL0 group rates) at startup via `--add-param-file`, and the host copy carries heartbeat at 1.00 Hz plus position/GPS/attitude/status at ~4 Hz | Keep the SITL copy self-sufficient for the streams the core consumes when the stack or its service inventory changes, and re-run the scenarios when either moves |

## Official CONOPS gap analysis

Requirement IDs resolve in the [PRD source inventory](conops-requirements.md).
Every row is open. Existing primitives or mocked tests are partial implementation
coverage, never end-to-end competition acceptance. Dependencies are prerequisites,
not authorization to fly. Major gate evidence is expanded below this table.

| Gap / requirements | Required behavior and current source coverage | Owner | Dependencies | Safety risk | Objective test / required evidence | Gate | Blocked decision |
|---|---|---|---|---|---|---|---|
| GAP-01 / U-CORE-01, U-ADAPT-01 | One authority; main.cpp creates UDP connection, ROS node owns another Vehicle, plugin starts CLI and writes directly | C++ runtime / client adapters | G-M, versioned client contract | Conflicting actions and stale authorization | Concurrent client/pilot takeover/replay tests; one accepted writer, no automatic resume | G2 | D02/D10 |
| GAP-02 / AE27-NET-002 through AE27-NET-006 | Required telemetry fields; state.hpp lacks per-field age, position accuracy, link metrics and official mode model; protocol.cpp supplies MSL/home-relative altitude | C++ telemetry / link adapter | Firmware sources, time and datum model | False position/AGL/health | Independent fixtures for terrain change, fresh heartbeat with frozen position, unknown accuracy/battery/link and overlapping modes | G2/G4 | D08/Q04 |
| GAP-03 / AE27-NET-001, AE27-NET-009 through AE27-NET-014 | Authenticated armed 1 Hz upload and scoring receipt; no competition adapter exists; MAVLink heartbeat is unrelated | Competition adapter | GAP-02, official token/schema contract | Hidden outage or blocked control | Independent receiver clocks, 300 s armed run, 30 s+ outages, cadence boundaries, expired auth, startup armed; official server acceptance | G4 | Q04/D11 |
| GAP-04 / AE27-NET-007/008, AE27-INT-002 | Receive 1 Hz traffic and avoid cylinders; no traffic model or advisories | Adapter / C++ safety / operator | GAP-02/03, approved cylinder semantics and response budget | Collision / stale feed interpreted clear | Crossing/head-on/overtaking, vertical separation, exact boundary, duplicate IDs, stale/reordered feed; demonstrate actual operator avoidance within budget | G4/G7 | Q04/D05/D08 |
| GAP-05 / AE27-OPS-005/006/020/037/038 | Continuous all-mode concave/altitude containment; geofence.cpp checks position targets only, unset fence allows targets; fence upload not automatic enforcement proof | ArduPilot / C++ safety | Verified hard polygon, internal inset distance, AGL source, firmware | Flyaway / wrong termination trigger | Reject absent/invalid competition fence; readback action/altitude; trajectory crossing despite inside endpoint; breach tests all modes | G2/G7 | U-FEN-01 resolved; hard-fence/AGL evidence still required |
| GAP-06 / AE27-OPS-015 through AE27-OPS-019/035 | Aircraft-specific termination always available, five-second entry and C2-loss self-termination; watchdog zero and plugin LAND dispatch insufficient | ArduPilot / safety hardware; C++ verifies readiness | Qualified termination path, phase/type detection | Catastrophic uncontrolled aircraft | Props-off judge demonstration plus isolated SITL fault matrix; independently measure fixed-wing outputs, rotary descent/touchdown, C2/core/power faults and transition phases | G7 | Q02/D01/D09 |
| GAP-07 / AE27-T1-001/002/008/012 | Full 3-5 km prescribed lap route, single-battery reserve, VTOL landing; executor.cpp only synchronous steps and Copter mode constants | C++ mission / ArduPilot | G-M QuadPlane parity, GAP-05/06 | Invalid transition, exhaustion, false landing success | QuadPlane takeoff/transition/course/laps/abort/landing SITL, then single-battery field rehearsal with independent position/landed evidence | G5/G7 | D01/D08; actual course |
| GAP-08 / AE27-T1-003 through AE27-T1-007/010/011 | Deer totals/clusters/codes/anomalies and timed TXT; video bridge moves images, no task perception/export | Python perception / C++ mission / ground evidence | Capture hardware, calibrated imagery, clock/attempt model | Wrong target or misleading count | Held-out tagged decoys/distractors/occlusion, clustering oracle, timed upload name/content checks and operator audit | G5 | Q05/D01/D08/D11 |
| GAP-09 / AE27-T2-001 through AE27-T2-006/017 | Custom compliant tracker and single attachment; no tracker interface, generic outputs only | Tracker hardware / payload / C++ mission | Chosen device/mechanism and safe feedback | Duplicate release, wrong target, contact harm | Complete tracker mass/axis measurement, placement witness, duplicate/reboot/no-contact/jam tests; 100 m exclusion through sampling/return | G6/G7 | D04 and official site geometry |
| GAP-10 / AE27-T2-007 through AE27-T2-010/018 | Five-minute path/ISO8601 CSV on time; no tracker ingestion/export | Tracker adapter / evidence tools | Device, time synchronization, GAP-09 | Lost person offset / falsely precise path | Independent 5 m/s trajectory with burst/gap/reboot faults; linear interpolation 5/15 m error scoring and deadline upload | G6 | Q07/D04/D08 |
| GAP-11 / AE27-T2-011 through AE27-T2-015/019 | Intact egg, 1-4 cm green core, four spheres on pad; release_payload interlock not collection verification; raw output bypass exists | Payload hardware / C++ safety | Mechanism/feedback, reserved channels, authority | Unintended actuation, damaged sample | Independently inspect quantity/integrity on pad; power/link/jam/unknown feedback, no repeated irreversible action | G6/G7 | D04/Q07 |
| GAP-12 / AE27-T2-020/021; AE27-INT-001 | Optional no-intervention bonus; conditional swap recovery; no autonomous task/persisted resume | C++ mission / operator | GAP-09/11, explicit strategy | Unsafe autonomous interaction / replay | Continuous no-intervention success record if bonus selected; swap/resume permissions expire and records persist if swaps permitted | G6 | D05/Q03/Q06 |
| GAP-13 / U-PROF-01 through U-PROF-03; AE27-OPS-004/027/029 | Core viable in all profiles with truthful missing features and live map; templates tested, capability service/capture/resource evidence absent | Integration / Mission Planner | Qualified camera/radio/compute, single owner | Frozen video/map or overload starves safety | Clean boots without GPU/ROS/camera; source loss and saturated video/LTE/RTK; position-age display and measured deadlines | G3/G7 | D01/D02/D06/D08/D09 |
| GAP-14 / AE27-OPS-023 through AE27-OPS-036 | Physical aircraft, mass, electric power, RF, prop inhibit and FRR | Airframe / safety / flight leads | Final hardware and approved procedures | Unsafe/ineligible aircraft | Per-aircraft weigh/BOM/licence/prop-inhibit inspection, full proof-flight video, weather/energy envelope and approved FRR | G7/G8 | Q03/Q08/D01/D10 |
| GAP-15 / AE27-ADM-001 through AE27-ADM-035; AE27-OPS-022 | Deadline, eligibility, publication, preparation and attempt evidence; no competition deliverable workflow | Competition lead / ground evidence | Roster, owners, secure storage and reviewed rubric | Lost eligibility/evidence, mixed attempts | Timed isolated-crew rehearsals, attempt reset, file/heading/page/rubric and private receipt checks | G8 | D10/D11/Q08/Q09 |
| GAP-16 / U project MAVSDK decision | Production MAVSDK mandatory; main.cpp still UdpMavlinkConnection, optional smoke only | Transport lead | Phase A pins/licences/CI/SITL/budgets then parity | Mistaking telemetry smoke for safe control | Phases A-E evidence, Copter and QuadPlane, watchdog/stop/heartbeat/fence/parameters, production provenance and rollback | G-M | D08/D10; resource approval then parity |

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
open despite stronger wording elsewhere. Hosted run `34648914427` at NOMAD
`6a3e970` on 2026-09-11 resolved the zero-delivery discrepancy: an independent
pymavlink observer recorded the ordered nonzero then all-zero setpoints, and the
vehicle held hover afterward. The full run later stopped at SR-LNK-04 because the
GCS-heartbeat harness measured 2.5 Hz against its documented 1 Hz ceiling, so the
remaining scenarios and containment gate stay open. No hardware evidence was
supplied for this review; hardware availability beyond the user's stated
selections is not inferred.

## Gate sequence and accountable roles

Roles below need named owners (D10). Dependencies order implementation, not
permission to operate hardware. No gate is closed merely by this planning change.

### G0 — Planning baseline (lead + competition lead)

Reconcile these docs, preserve stable safety IDs, resolve links and traceability.
The full v1.0 CONOPS is now read and inventoried under AE27 IDs in the PRD
source appendix. Preserve the retired P-ID crosswalk; retain resolved Q01/U-FEN-01, resolve Q02-Q09 and assign
named owners without pretending this review closes organizer questions.
Exit: strict docs build and traceability test pass; no unsupported requirement
claims; assigned decision owners and resolution of blocking interpretations
remain necessary. This pass establishes provenance, not flight readiness.

### G1 — Executable baseline (build/integration lead; depends on G0 planning)

Repair C02/C10-C12 deployment/task leftovers without reintroducing Edge Core.
Retain video and both optional compute placements. Demonstrate fresh checkout
builds and selected simulation startup with no deleted-path dependencies.
Exit: test-core, test-python, lint, format-check, complexity-check, docs-build,
plugin compile/helper checks and supported ROS build/tests. Check command paths
against actual files; current runtime repairs require their own focused change.

#### C02 repair evidence - 2026-09-09

Baseline commit: `d31b0aa`; subsequent runtime-repair topic work is recorded in
repository history. The repair removes deleted Edge Core services, image jobs,
console entrypoint and missing API/gimbal test tasks. `dev`/`dev-build` build the
core; `sitl` builds it before simulator startup. Compose forwards host traffic
independently of ROS, with the ROS destination enabled by `sim-ros-up`. The SITL
CI default no longer duplicates the private relay output. ROS image/module paths
package the retained video tool. Incomplete perception/Gazebo startup tasks exit
unavailable before starting containers; optional images and media/compute source
remain.

Local evidence: CTest 9/9; `pixi run test` 258 passed, 3 environment skips;
retained Python coverage 96.65% against the unchanged 85% floor. Seven runtime
wiring cases cover packaging metadata/imports, workflow task references, Compose
paths and absent-provider failure. A setuptools wheel was built, extracted
outside the checkout, and imported under isolated Python; video `--help` passed.
Daemon-free Compose resolution passed for SITL, ROS and Gazebo scaffolding, with
expected unique SITL/ROS output destinations. The unavailable perception task
returned exit 1 with its stated reason. Lint, format, type-check, complexity,
strict docs and all changed-file pre-commit hooks passed. Whole-tree pre-commit
remains blocked by the pre-existing missing SPDX header in
scripts/hardware/servo_test.c; that
unrelated hardware file is unchanged.

Docker daemon unavailable: no image build, live SITL, ROS integration, sensor
stream or GPU/hardware qualification was run for this repair. G1 is not closed.
Python dependency pruning is deferred; the distribution
name `nomad-edge` remains for compatibility although its deleted CLI is removed.

#### C10-C12 repair evidence - 2026-09-09

The three product profiles now use explicit C++ UDP listener endpoints, contain
no development key, and leave optional ROS/video/container workloads disabled
until their providers are qualified. Loading and Mission Planner synchronization
validate profile identity, endpoint scheme, host and port before mutation.
Switching profiles preserves unrelated plugin settings, clears empty owned
settings and removes retired Jetson fields. The shell profile command delegates
to the validated Python implementation.

The deployment CLI and systemd inventory no longer reference the deleted Edge
Core service. The retained video service directly owns its in-container Python
process, and the Jetson image includes that module. Its unauthenticated control
HTTP endpoint accepts loopback binds only; wildcard and remote binds fail before
the media pipeline starts.

No image, service or hardware runtime was started. Optional compute, camera,
estimator, RTSP output and clean-boot systemd behavior remain unqualified.

Local evidence: 297 Python tests passed with 3 skipped and 96.34% coverage;
all 9 CTest targets passed; strict documentation, lint, formatting, type checks,
complexity limits, lock validation, changed-file pre-commit hooks and edited
shell syntax checks passed.

#### Setup and provisioning repair evidence - 2026-09-09

Development bootstrap now builds the C++ core and checks the product profile
manager. Local and remote Jetson setup load or validate a supported product
profile, build the core, install the retained service inventory and reject
unqualified optional autostarts. Tailscale and Jetson firewall setup no longer
opens the deleted HTTP API or grants blanket access on the Tailscale interface.
Remote setup uses the existing SSH known-hosts trust store and rejects unknown
host keys. No remote command, package installation, firewall mutation, service
start or hardware check was run; these paths have structural and mocked-command
evidence only. Local evidence after integration: 313 Python tests passed with
3 skipped and 96.34% retained-module coverage; all 9 CTest targets passed; lint,
formatting, type checks, strict documentation, complexity, lock, shell syntax,
PowerShell parsing and changed-file pre-commit checks passed.

### G-M — MAVSDK adoption (transport lead; early prerequisite after G1)

Complete Phase A build/dependency/license/SITL evidence, then command, velocity,
watchdog/zero, GCS heartbeat and fence parity. Preserve unit fault coverage,
independent wire observations and CLI/plugin compatibility. Test both Copter
and the chosen QuadPlane firmware as support is introduced; existing Copter
tests alone cannot qualify Task 1.

Create focused, reviewable implementation changes with unit tests, integration
evidence, requirement mapping and limitations. Under the 2026-09-12 ownership
split, ArduPilot command, mode and telemetry semantics are delivered in the pinned
fork and NOMAD does not grow new ArduPilot paths; fork patches need reproducible
tests and separately tracked upstream changes when publication is authorized.

Exit: phases A-E in [MAVSDK adoption](mavsdk-adoption.md) pass; default production
runtime demonstrably uses MAVSDK; legacy deletion passes gates below; transitive
notices and supported firmware matrix are recorded. Phase F upstream acceptance
may lag, but required patches must be maintained and reviewed. G-M is mandatory
for G8 and precedes dependent integrated competition command work; isolated
server/CV prototypes may proceed without waiting.

### G2 — One authority and aircraft semantics (core + safety leads; G1/G-M)

Add the small persistent runtime and client boundary; unify mission/payload/
traffic state, stop/cancel/restart semantics and audit. Correct C04-C09/C13/C15.
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

Implement the confirmed field model and an independent fixture, then the
official schema adapter after Q04. Send 1 Hz telemetry whenever armed in Task 1
and consume 1 Hz traffic; event uploads and Task 2 applicability are unresolved. Define data age, retry, queue, clock and outage
policies with approved thresholds. Advisory-first is the user's initial scope;
manual operation is permitted, but actual exclusion-zone avoidance must be
demonstrated. Automatic maneuver design is not mandated by this document.

Exit: record send and independent receipt timestamps, missed cycles and jitter;
startup/armed and clock/rate penalties are independently exercised; malformed/auth-expired/
rate-limited/offline server cannot block command handling. Inject crossing,
head-on, overtaking, diverging, stationary, duplicate-ID and stale traffic with
ground truth; measure false/missed advisories and warning time against approved
thresholds. Missing feed shows unknown. Measure 300 seconds of armed receipt, 30-second
and longer outages, >1.1-second and <0.4-second intervals, invalid GPS and startup
armed against clarified scoring semantics. Test cylinder radial/vertical boundaries
and operator response under load. Official-server acceptance and actual simulated
traffic avoidance are separate runs, not mock-only closure.

### G5 — Task 1 mission (mission + perception + flight leads; G2-G4)

Cover area planning, CV evidence, count/identity review, simulated cooperation,
traffic advisories and safe return/landing on the VTOL. Characterize video
capture access from Walksnail equipment before assuming frame availability.
GNSS/Here4 RTK is the intended navigation baseline; VIO is not a Task 1 dependency.

Exit: independent known animal/marker counts and identities, held-out imagery,
coverage and geolocation errors, duplicate/occlusion cases and review audit.
QuadPlane SITL plus authorized field rehearsal verifies complete task with
single-battery reserve, transitions and landing. Use the prescribed 3-5 km total lap course, antler/no-antler decoys, capitalized
two-character codes, 10 m clusters (Q05 oracle), anomaly descriptions and correctly
named TXT export. Test submission exactly at five minutes before and 15 minutes
after cutoff and adjacent timestamps. Record actual Drive receipt. Runtime
accuracy/reserve/latency budgets still need D08; source scoring does not set them.

### G6 — Task 2 payload mission (payload + mission + safety leads; G2-G4, D04)

Custom tracker technology (possibly ESP32), attachment and sample mechanism remain
undecided. Implement tracker identity/position ingestion, association and mapped
path with uncertainty/gaps. Explicit action authorization is initial scope;
autonomous sampling is an optional scored strategy requiring D05/Q06 and
no-intervention evidence for the complete pickup-to-pad sequence.

Exit: known ground-truth trajectory and measured error/age; wrong tracker,
packet loss, replay, reboot, RF interference and depleted tracker battery cases.
Observe tag attachment and simulated sample collection with independent feedback,
measure quantity/containment, and test jam/no-contact/relay-off failure/abort.
Measure tracker under 250 g and at most 8 cm per axis, exactly one placement,
100 m withdrawal and continued horizontal offset, five-minute path up to 5 m/s,
chronological ISO8601 CSV before cutoff and independent interpolated 5/15 m
accuracy scoring. Inspect intact egg, 1-4 cm cylindrical core at least 75% green,
and four intact spheres on the 32-inch pad by cutoff. Require observed safe
landing and field clearance except the attached tracker. Q07 governs unspecified
measurement details. If Q03 permits battery swaps, prove disarm, payload safety,
persisted records, expired permissions and explicit resume without duplicate action.

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
verify the independent all-mode termination mechanism, including loss of C2,
within-five-second activation entry, fixed-wing motor/surface state, rotary
minimum-2-m/s descent through touchdown and QuadPlane transition cases. Settle
Q02 before aircraft testing; fence/altitude/termination readback alone does
not prove behavior. Complete every AE27-OPS FRR, licence, propeller-inhibit,
weather and proof-flight requirement, with judge acceptance before competition.
Authorized field tests cover aircraft-specific abort, transition failure, landing, traffic procedures and safe payload.
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

### MAVSDK Phase A dependency hardening - 2026-09-10

Requirement: project MAVSDK decision / GAP-16. Falsification: change the
PicoSHA2 commit, either archive SHA-256, extraction timestamp option or a bundled
licence text and require the provenance tests to fail; rebuild and run the peer
fixture to reject wrong/absent systems.

The project MAVSDK fork is published and NOMAD pins
`9884f109533f564bc6250e5471e6301d3a62f4a7`. The selected graph pins PicoSHA2 to
`1bf940d8a03bb752604fbb366d47b97b50b9e6ce`, verifies nlohmann JSON and XZ
archives with SHA-256, handles archive extraction on both legacy and newer CMake,
and uses the pinned nested pymavlink generator source rather than build-time
network package resolution. `licenses/mavsdk-phase-a/` carries the selected-build
licence texts; the provenance checker verifies reviewed revisions, hashes, patch
invariants, NOTICE coverage and licence content.

Recursive hosted test run `34535620056` passed the Python suite, C++ core,
optional MAVSDK build, provenance checker and deterministic expected/wrong/absent
peer qualification on both Ubuntu and Windows. Selected ROS-image run
`34538394497` built with NOMAD_ENABLE_MAVSDK=ON and passed the ROS adapter tests.
Mainline SITL run `34538903820` built ArduPilot Copter 4.7.1, brought up the
development/SITL stack and passed the live MAVSDK Phase A connect/status smoke.
The larger SITL loop-closure job is intentionally nightly/on-demand; the main
push ran only the reduced Phase A live regression gate.

Historical Windows measurements remain diagnostic: the warm build tree measured
379,308,757 bytes and the smoke executable 2,032,128 bytes. They are not approved
budgets. Repeatable current build-tree, executable, runtime memory, startup and
CI-time measurements plus explicit budget approval remain the sole Phase A gate
items. Production continues to use the legacy transport; command/telemetry parity
and cutover belong to later G-M phases.

### MAVSDK ArduPilot ownership - 2026-09-12

Requirement: project MAVSDK decision / GAP-16 and G-M. Falsification: introduce a
new ArduPilot-specific command path, mode interpretation or telemetry
interpretation inside NOMAD that the pinned fork should own, and require review to
reject it; or change a fork patch without a provenance and requalification update.

Decision (user-confirmed): the pinned fork owns ArduPilot command, mode and
telemetry semantics; NOMAD keeps safety policy, validation, authoritative
verification, deadlines, audit and the client contract. This widens the scope
recorded in [MAVSDK adoption](mavsdk-adoption.md), which had limited the fork to
dependency patches and left the gaps in NOMAD's adapter. The measured gap list at
`9884f109` sits in that document, and the raw-frame paths in
`MavsdkMavlinkConnection` become transitional rather than the intended shape.

Sequencing effect: the ArduPilot completeness work in fork Phase F is prerequisite
work for Phases B-D, not post-cutover maintenance. Unknown work remains large —
mode setting, relative-altitude location commands, the output verbs and the
telemetry parity fields are all unstarted in the fork, and no fork change has been
made beyond dependency pinning.

Evidence effect: fork code becomes safety-relevant code outside this repository.
Each patch needs its own test, an independent wire or SITL observation against the
selected firmware, provenance pinning and an upstream change when publication is
authorized. Existing NOMAD verification evidence is unaffected because outcome
authority does not move.

### MAVSDK Phase B transport parity - 2026-09-11

Requirement: project MAVSDK decision / GAP-16 (command parity). Falsification:
run `pixi run test-mavsdk-phase-b`; a denied ACK, a missing ACK or a wrong
autopilot system ID must make the CLI or probe report failure (never success),
and `goto` must leave the host as COMMAND_INT with the GLOBAL_RELATIVE_ALT_INT
frame.

The opt-in MAVSDK transport now implements `MavlinkConnection` against the
reviewed pin `9884f109533f564bc6250e5471e6301d3a62f4a7` (the submodule was
reconciled from a dirty newer checkout, and provenance passes again). Commands
are issued as raw COMMAND_LONG/COMMAND_INT so NOMAD keeps its MAVLink
result-code contract and relative-altitude command frame. The CLI gains
`--transport udp|mavsdk` (default udp) and `--system-id`; the legacy codec is
untouched and remains the default.

Local evidence (2026-09-11, MSVC Release, Copter-oriented peer fixture):
`nomad_mavsdk_connection_tests` passes its no-peer configuration and
fail-closed cases; `scripts/dev/mavsdk_connection_fixture.py` passes status
telemetry, accepted, denied, timeout, COMMAND_INT frame 6, COMMAND_LONG and
wrong-identity cases, and per-command parity for mode, takeoff, goto, land, RTL,
servo, relay, gimbal-config and user-command. The command cases drive the real
CLI against a peer whose initial state is observably different from each
command's required result, so a pass means the core's state verification
observed the change rather than trusting the acknowledgement. motor-test parity
was added once C23 was fixed, and asserts the corrected id on the wire.
mission parity remains open, so this is transport parity evidence and not a
closed Phase E gate; the live SITL runs below complete Phase B's stated exit for
Copter without closing G-M.

Link parity was added on 2026-09-12 by porting the legacy transport's link-level
cases onto the same peer fixture:

- `case_gcs_heartbeat_announces_to_a_silent_peer` points NOMAD at a bound,
silent peer with `udpout:` and requires the announcements to arrive while no
vehicle is latched: four GCS heartbeats (`MAV_TYPE_GCS`,
`MAV_AUTOPILOT_INVALID`, component 190, system 245) in a 3 s discovery window
that discovers nothing. This is the pre-latch announcement a heartbeat-gated
relay needs, and the behavior `safety/gcs-heartbeat-cadence` protects. A
`udpin:` link has no pre-latch destination at all, because MAVSDK only learns
one from traffic it receives, so a relay setup must name the relay with
`udpout:` — the successor to the legacy `NOMAD_RELAY_ADDRESS` override, which
the MAVSDK transport does not implement separately.
- `case_coalesced_telemetry_is_verified` drives the CLI against a peer that
joins every telemetry frame and the command acknowledgement into one datagram,
the shape MAVProxy and router links produce, and still requires the arm to be
verified from reported state.
- `case_link_loss_is_observed_as_stale` streams telemetry, stops it while the
socket stays open, and requires `heartbeat_fresh` and `connected` to become
false through the probe's `--staleness` mode. That is the link-observation half
of SR-LNK-01.

Phase C (velocity) landed in the same pass. `send_velocity` queues
SET_POSITION_TARGET_LOCAL_NED through MAVSDK's passthrough using
`queue_message()` (so MAVSDK owns sequence numbering) with the legacy codec's
body-frame mask 0x07C7 and frame 9, refuses a non-finite rate, requires a live
latched peer for a non-zero setpoint, and zeros the vehicle before tearing the
link down. `case_velocity_setpoint_reaches_the_wire` proves all of that on the
wire rather than from the transport's own report: the peer decodes the frames it
received and the case requires the requested rates with that mask and frame and
the right target system, requires that no non-finite frame ever left, and
requires the all-zero setpoint after the stream when the transport disconnects.
Both guards were confirmed load-bearing by removing them and watching the case
fail, and the non-finite probe runs on a live link so a check that only held on
a dead link could not pass for an enforced one.

The core-level half of SR-LNK-03 followed in the same pass.
`tests/mavsdk_zero_delivery_test.cpp` is the MAVSDK successor to the legacy
`tests/zero_delivery_test.cpp`: it builds a real `Vehicle` on the MAVSDK
transport, feeds VIO, arms and selects GUIDED, then streams the setpoint. The
fixture drives five scenarios against the same peer — the watchdog stopping a
dead command stream, `Vehicle::stop_velocity()`, vehicle destruction
mid-stream, heartbeat loss, and VIO-feed loss — and each must report its own
watchdog reason while the peer observes the all-zero setpoint as the newest
setpoint on the wire. The binary also covers the no-link failure path under
CTest without a peer, so a declined command cannot masquerade as a stopped
stream. `zero_delivery_test.cpp` can therefore be deleted with the legacy codec
at Phase E instead of being ported under pressure; until then both proofs exist
and the SITL scenario `core-sitl-velocity-watchdog` covers the same behavior on
the default transport.

Phase D (fence/params) landed on 2026-09-12 and closes the last functional gap
before the cutover decision. MAVSDK's `Geofence` and `Param` plugins implement
the transport's fence and parameter surface, and the two plugins are enabled in
the CMake MAVSDK options (`ENABLED_PLUGINS telemetry geofence param`) — without
that the plugin headers compile but the library link fails. Single-vertex fence
transfer (`send_fence_point`, `request_fence_point`, `wait_for_fence_point`)
stays fail-closed, because MAVSDK moves whole polygons and the core only ever
uploads a plan and then reads the autopilot's own copy back, so a partial
transfer would have no caller to serve.

`case_fence_upload_and_readback` and `case_disabled_fence_does_not_verify` are
the SR-FEN-01 evidence on this transport. They drive a real `Vehicle` through
`upload_fence` and `verify_fence_uploaded` with the probe's `--fence` mode
against a peer that stores the fence mission items it is sent, so the polygon
the peer decoded is the proof the upload happened and the readback cannot come
from the transport's own bookkeeping. The enabled case requires the verified
result plus the refusal paths — a boundary below the three-vertex minimum, a
non-finite vertex, and an expected boundary the autopilot does not hold. The
disabled case requires the upload to succeed while verification fails, because a
plan on the autopilot is not an enforced fence: `verify_fence_uploaded` reads
`FENCE_ENABLE` back and fails closed when it is not 1. Both guards were confirmed
load-bearing by breaking them and watching the case fail — a fabricated
`FENCE_ENABLE` made the disabled case verify wrongly, and a no-op upload left
the peer holding no polygon.

One fabricated success was found and fixed while reviewing the cutover surface.
`MavsdkMavlinkConnection::request_data_stream` returned `is_connected()`, so it
reported success for a `REQUEST_DATA_STREAM` it never put on the wire, while the
legacy transport genuinely encodes and sends that frame. It now queues the
frame through MAVSDK's passthrough with the legacy fields (stream id, requested
rate, `start_stop = 1`) and requires a live latched peer, so a request on a dead
link fails closed. `case_data_stream_request_reaches_the_wire` proves both
halves: the peer decodes the frame and the case requires stream 1 at 4 Hz
targeting the autopilot. The case was confirmed load-bearing by restoring the
fabricated success, which reported `requested=1` while the peer decoded nothing.
No production caller requests a stream today — the core reads the telemetry
MAVSDK's own subscriptions deliver, and the SITL stack supplies stream rates
through `docker/sitl-streams.parm` (C24) — so this is transport-honesty parity,
not a behavior change on the current path.

What Phase D does not yet claim: `read_param` applies MAVSDK's own request
timeout instead of the caller's budget (the core's ten-second readback timeout is
not stacked on it), and no parameter write path exists because the core only
reads. Phase E (production cutover) remains open, and production still uses the
current codec.

One candidate fix was measured and withdrawn rather than shipped: setting
`Mavsdk::Configuration::set_always_send_heartbeats(true)`, which MAVSDK
documents as sending heartbeats without a connected system, changed nothing (four
heartbeats either way), so the transport keeps MAVSDK's `GroundStation`
default and the fixture pins the observable behavior instead.

SITL exit criterion met later the same day, after fixing the feed rather than
the transport. Two captures on passive vehicle links inside the container (no
host relay involved) delivered ~1 Hz heartbeats and no position/attitude stream
until a GCS sent `REQUEST_DATA_STREAM`, after which ATTITUDE/
GLOBAL_POSITION_INT/GPS_RAW_INT/SYS_STATUS arrived at roughly 5 Hz. ArduPilot
behaves that way by design and the core deliberately never requests streams, so
with the Python edge service deleted nothing requested them for the host-side
copy the scenarios read; see C24. `docker/sitl-streams.parm` now sets
`SR0_POSITION`, `SR0_EXT_STAT`, `SR0_EXTRA1` and `SR0_EXTRA2` at SITL startup,
and a 20 s capture of the host copy then delivered HEARTBEAT at 1.00 Hz with
GLOBAL_POSITION_INT, GPS_RAW_INT, ATTITUDE and SYS_STATUS each at ~4 Hz.

With that feed: the legacy baseline passes `pixi run core-sitl-status`,
`core-sitl-command-flow` and `core-sitl-payload`, and the MAVSDK transport
passes the command flow (GUIDED mode, 3D GPS fix, arm, takeoff to 5 m, guided
goto sent as COMMAND_INT, RTL, land, disarm, every step verified against
reported state) and the payload acceptance run with `NOMAD_TRANSPORT=mavsdk`.
One earlier observation is withdrawn: this document first recorded the
container-to-host UDP relay as losing roughly three quarters of datagrams. That
reading came from the pre-fix stack, whose container had been running six days
with a stale environment and was still initialising when captured; the
recreated stack delivers heartbeats at exactly 1.00 Hz, so no relay loss is
claimed and the cause of that earlier rate is left unexplained rather than
guessed. Remaining Phase B gaps are unchanged: vehicle-class identification and
QuadPlane coverage. Phases C, D and E stay open, and production still uses the
current codec. C23 (the undefined motor-test command id) is fixed and carries
live evidence: on 2026-09-12 against Copter 4.7.1, `nomad motor-test 1 0 0.1`
sent `COMMAND_LONG cmd=139` and ArduPilot answered `result=3
(MAV_RESULT_UNSUPPORTED)`, and after the id was corrected the same run sent
`cmd=209` and received `result=0 (MAV_RESULT_ACCEPTED)` with the vehicle's own
"starting motor test" / "finished motor test" status texts, which also confirms
the parameter layout (`param2` is the throttle type). That acceptance is a
command acknowledgement and a status text rather than an independent state
measurement, so the verb stays out of the state-verified evidence set.

Full legacy sweep on that feed (2026-09-12, Copter 4.7.1): `core-sitl-status`,
`core-sitl-command-flow`, `core-sitl-mission`, `core-sitl-velocity-watchdog`,
`core-sitl-geofence`, `core-sitl-payload`, `core-sitl-link-loss`,
`core-sitl-link-recovery`, `core-sitl-zero-delivery`, `sitl-scenario` and
`sitl-fence` pass. Two harness defects surfaced while running them, neither in
the core. `sitl-fence` could never pass: it was the only scenario task without
the development API key, and every step is an actuation command the CLI refuses
without one (`audit command=... result=refused auth=none
reason=missing_api_key`); the task now sets it like its siblings. Separately,
the geofence scenario leaves its uploaded polygon fence active and ArduPilot
then refuses guided targets outside that polygon (observed: the same reposition
accepted with `FENCE_ENABLE=0`, rejected with the polygon loaded), so
scenario ordering now matters and is recorded in `tests/sitl/README.md`.

`core-sitl-gcs-heartbeat` passed once the transport kept announcing for the whole
wait: its negative control requires at least two captured announcements, but the
UDP transport announced once and then blocked in a receive until the deadline, so
a closed gate yielded exactly one. The receive is now capped at a short slice
(measured 2026-09-12: the closed gate captured three announcements at ~1.0 Hz
before status failed closed, and the open gate relayed the stream). `SR-LNK-04`
still asks for MAVSDK requalification of this requirement, which remains open.

### Boundary clarification - 2026-09-10

The project owner resolved Q01: hard-boundary violation means termination;
soft boundary is an internal configurable inward distance from the same hard
polygon, e.g. 5 m. The existing plugin implementation/default inset is retained
unchanged, including its configuration controls. No second official soft polygon
or organizer ruling is required for this design. Appendix C's inconsistent labels
remain a source note. Hard-containment/termination tests and the separate 100 m
AGL/default-display discrepancy remain open; no runtime behavior changed.

### CONOPS reconciliation - 2026-09-10

Read all 36 pages before editing; inventoried 125 statements/conditions with
source pages, owner, safety relevance, provenance status and evidence. Visually
checked the contradictory Appendix C table and task scoring/sample tables.
Mapped implementation into 16 open gaps and retained stable SR and preview IDs.
The new structural test does not prove semantic completeness or flight compliance.

Checks recorded when the reconciliation itself completed, before the later
Phase A source hardening above:

- `pixi run test-core`: 10/10 CTests passed, Windows Debug; compiler emitted
  existing C4530 exception-unwind warnings. No C++ source/build options changed.
- `pixi run test`: 328 passed, 3 skipped; retained Python coverage 96.34%.
  This runs the full Python suite, including the four new traceability cases.
- Focused CONOPS and safety traceability suite: 74 passed.
- `pixi run lint`, `format-check`, `typecheck`, `complexity-check`, `docs-build`
  and `check-mavsdk-phase-a`: passed. Type check covers its configured seven
  source files; it is not a C++ or full adapter proof.
- `pixi run lint-plugin`: non-deploying Release compile/dead-code check passed.
- All changed-file pre-commit hooks passed. Read-only whole-tree SPDX hook
  still fails only on the pre-existing scripts/hardware/servo_test.c header;
  that file is unchanged. A whole-tree all-hook pass is not claimed.
- Local Markdown link check: 67 links in changed documents resolved before
  this evidence note; every changed file below 100 KB. Final diff review includes
  both new files; no runtime/configuration mutation, secret, real deployment
  address, generated artifact or unrelated source change is included.

Changed document set: PLAN.md, TODO.md, README.md, CONTRIBUTING.md; docs/prd.md,
docs/conops-requirements.md (new subordinate appendix), docs/architecture.md,
docs/migration.md, docs/operations.md, docs/safety.md, docs/development.md,
docs/index.md, docs/mavsdk-adoption.md, docs/mavsdk-dependencies.md;
config/profiles/README.md, mission_planner/README.md and tests/sitl/README.md.
Supporting changes: properdocs.yml navigation and tests/test_conops_traceability.py.

The CONOPS reconciliation was prepared from clean main at `fab9f46`; those local
checks predate the later published implementation commits. Main now contains the
reconciliation plus subsequent MAVSDK Phase A hardening and qualification. Do
not reinterpret the earlier local pass counts as current hosted or live-SITL
evidence; the current Phase A evidence is recorded separately above.

No hardware or official-server acceptance was performed by the reconciliation.
Q04 remains open. MAVSDK production adoption remains incomplete; no release gate
closes solely because documentation, hosted Phase A smoke, or local regressions
pass.

### Earlier planning review - 2026-09-08

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
