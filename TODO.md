# NOMAD work ledger

One ledger; one active item. Requirements/decisions live in [PRD](docs/prd.md),
implementation/evidence in [migration](docs/migration.md), and transport details
in [MAVSDK adoption](docs/mavsdk-adoption.md).

Status: [ ] open, [~] active, [x] complete. A task being implemented does not
close its integration or release gate.

## Current work

- [ ] G-M Phase A resource qualification: the published MAVSDK graph now has
  recursive clean-checkout provenance/build evidence, hosted Linux/Windows/ROS
  qualification and a live ArduPilot Copter SITL connect/status pass. The build
  task now records configure/build timing and footprint metrics, while live smoke
  records per-process-tree startup/RSS metrics; hosted jobs retain both artifacts.
  A clean local Windows checkout at the reviewed gitlink passes provenance,
  build, core/Python suites and deterministic peer cases. Retained hosted samples
  include Linux build-tree `216712508` bytes, Windows build-tree `446186671`
  bytes, Linux/Windows smoke executables of `4753832`/`2025472` bytes, Linux
  runtime peak RSS of `10297344` bytes, and a full hosted test run of about 18
  minutes. The merged-main SITL run retained a Linux build-tree sample of
  `216679160` bytes and runtime peak RSS samples of `10297344` and `9912320`
  bytes. These are observations, not approved budgets: the gate is explicitly
  deferred until a named owner approves build-tree, executable, memory, startup
  and CI-time limits and the retained artifacts are compared with them.
  Falsification: the reviewed pins stop reproducing, live qualification regresses,
  or measured resources exceed an approved threshold.

- [~] G-M aircraft-class support slice: heartbeat identity is now classified into
  ArduPilot Copter, Plane, QuadPlane or Unknown, carried in core vehicle state,
  and used for guided/landing/RTL mode semantics and body-velocity admission.
  Unknown and unsupported identities fail closed, while fixed-wing body-frame
  velocity is rejected. Focused CTest coverage passes; Plane/QuadPlane SITL,
  Task 1 VTOL execution and the complete supported-aircraft release matrix remain
  open. Falsification: an unsupported identity is admitted, a class-specific
  mode is not verified, or the focused and hosted regression suites diverge.

- [x] G2 / SR-LNK-03 zero-delivery evidence: repair the live observer's MAVLink
  datagram parsing, prove nonzero-then-zero ordering independently, and rerun the
  complete hosted Copter SITL suite. Merged-main run
  `35489428247` passed the full matrix at `26d7f9b`; this repairs evidence
  collection only and does not weaken the core watchdog or substitute command
  acknowledgements for wire evidence. Falsification: the observer misses a valid
  packed setpoint, accepts zero without a preceding nonzero command, or live wire
  capture remains empty.

- [x] G2 / SR-LNK-04 GCS-heartbeat evidence: validate at least three measured
  intervals between 0.9 s and 1.3 s, route MAVSDK's `udpout:` source through the
  heartbeat-gated relay, preserve the dropped-announcement negative control, and
  rerun the current-head full SITL suite. Merged-main run `35489428247` passed
  the heartbeat gate and every later Copter scenario at `26d7f9b`. Falsification:
  cadence exceeds the owned limit, the relay opens without valid GCS heartbeats,
  or a future complete suite stops at this gate.

## Ordered implementation backlog

- [ ] G0: obtain organizer rulings Q02-Q09 and assign named decision/gate owners
  (D10). The 125-entry documentation reconciliation and Q01 decision are complete;
  unanswered interpretations keep the planning gate open.
- [x] G-M Phases B–E implementation: the narrow MAVSDK connection owns the
  existing core boundary, command/telemetry/velocity/watchdog, GCS heartbeat,
  zero-delivery, fence and parameter paths; the default production runtime uses
  it and the hand-written codec is removed. See the implementation inventory and
  cutover status in [migration](docs/migration.md). This records source and
  focused-test completion, not release qualification.
- [ ] G-M current-head integration and release gates: qualify supported vehicle
  classes (including QuadPlane), rerun the full current-head ROS/SITL matrix,
  and complete packaging/install/rollback and G8 evidence. Falsification: any
  required adapter or SITL gate fails, or the release artifact cannot be
  installed and rolled back safely.
- [ ] G-M: add ArduPlane/QuadPlane support coverage for Task 1 alongside Copter;
  the first identity and class-specific mode slice is implemented with focused
  tests, but Plane/QuadPlane SITL and independent Task 1 evidence are still open.
- [ ] G-M Phase F: upstream tested ArduPilot fixes; track patches, review status
  and maintenance owner. Upstream acceptance is separate from local parity.
- [ ] G2 / C04–C09, C13, C15: single persistent command owner, authenticated client
  contract, fresh telemetry, aircraft-specific semantics, cancellation and
  truthful final outcomes; migrate direct plugin policy and hazardous outputs.
- [ ] G3: runtime capability and profile tests; Task 1 GPU-groundstation video/CV
  path, Task 2 optional onboard path, minimal operation without perception.
- [ ] G4: opt-in AEAC 2027 module: transcribe/version the official portal wire
  contract, then implement armed telemetry and traffic cylinders with an
  independent timing mock, demonstrated operator avoidance, outage/replay and
  official-server acceptance.
- [ ] G5: VTOL 3-5 km lap route, single-battery reserve, deer/cluster/tag/anomaly
  report and timed TXT upload, traffic avoidance and verified flight-line landing.
- [ ] G6: under-250-g/8-cm tracker, single placement and continued 100 m offset,
  five-minute CSV path, intact egg/core/spheres on pad, explicit authorization;
  autonomous bonuses only after D05/Q06; swap recovery conditional on Q03.
- [ ] G7: authorized hardware evidence for selected controller, GNSS/RTK, camera,
  RF paths, all-up mass/endurance, payload feedback and independent all-mode
  termination; verified hard fence and Q02 phase ruling, 100 m AGL and approved FRR.
- [ ] G8: full task rehearsals using MAVSDK, security/observability, clean
  packaging/rollback and release review.

Independent read-only research and isolated mock/CV prototypes may prepare later
gates. Do not mark multiple work items active or bypass predecessor safety gates.

## Existing slices to preserve

- [x] C++ library/CLI, MAVSDK MAVLink transport, telemetry and basic
  Copter vehicle/mission operations exist.
- [x] Velocity configuration, watchdog, VIO source validation, fence and
  dedicated payload safety paths with unit tests exist.
- [x] Wire-level zero-delivery tests and named SITL harnesses exist; hosted run
  `34648914427` independently observed the ordered nonzero/zero wire sequence and
  hover on Copter 4.7.1, and merged-main run `35489428247` passed the complete
  Copter matrix including containment. Aircraft-class, all-mode and hardware
  containment evidence remains open.
- [x] Mission Planner core client for goto/discrete outputs and ROS adapter exist;
  full command ownership migration remains open.
- [x] MAVSDK Phase A build/smoke target and fork wiring exist; the production
  cutover is implemented. Current-head integration and release gates remain open
  in G-M above.
- [x] MAVSDK Phase A selected-build inputs use immutable/strong-hash references,
  carry checked licence texts, and are published and pinned at the reviewed
  revision. Recursive hosted Linux/Windows, selected ROS and live Copter SITL
  qualification have passed; resource-budget approval is the remaining Phase A
  release blocker.
- [x] Three profile templates/config tests and retained Python video bridge exist;
  runtime and hardware capability evidence remains open.
- [x] Documentation review baseline: test-core 9/9; test-python 251 passed,
  3 skipped on 2026-09-08. See migration for subsequent quality checks.

## User decisions recorded on 2026-09-08

- Task 1: proposed lightweight VTOL, ground CV, no Jetson onboard; Pi Zero LTE
  backup and possible video transport.
- Task 2: heavier quad under the strict project 15 kg ceiling; optional Jetson if needed.
- Walksnail FPV camera; second camera TBD; no reliance on ZED.
- Flight-controller IMU; Cube Orange or custom ArduPilot controller; Here4 and
  RTK base selected in principle. Actual integrations remain unqualified.
- Custom tracker, possibly ESP32; payload mechanisms and feedback TBD.
- Traffic advisories and explicit action authorization first; revisit autonomy
  against the optional bonuses and actual traffic-avoidance obligation.
- MAVSDK is mandatory for competition and an early work priority with tests and
  focused reviewable changes. The CONOPS review changes documentation and
  traceability only.

## Decisions still open

See D01–D11 in PRD: exact aircraft/firmware and profiles, core host/IPC/remote
transport, capture and synchronized sensor access, tag/sample design, primary
radio, numeric performance/safety budgets, server contract, owners and evidence
retention. The "15 kg class" description does not override the below-15-kg rule.

## Review follow-through

- [x] Q01 / U-FEN-01: retain the plugin's configurable inward soft-boundary inset
  (e.g. 5 m) for internal use; hard-boundary violation triggers termination.
  No plugin changes or second official soft polygon required.

- [x] Read all 36 CONOPS pages and preserve source hash, exact section/page IDs,
  retired preview crosswalk, interpretation register and source-backed gap matrix.
- [ ] Competition lead: Q02 aircraft-phase termination; Q03 mass/swaps; Q04 server and
  traffic semantics; Q05 clustering; Q06 autonomy authorization; Q07 measurement/
  exports; Q08 final administrative package; Q09 scoring/window edge cases.
- [ ] Team: D04 attachment versus box and sample mechanisms; D05 bonus priorities;
  D01/D09 firmware, capture, radio and power; D02 core host/client boundary;
  D08 budgets; D10 named owners; D11 private evidence and credential lifecycle.

Keep source fixes separate: plugin EmergencyLand/fence parameter ownership,
AGL/accuracy/link-quality model, fixed-wing/QuadPlane semantics and physical
termination need requirement/fault-path tests before implementation. Do not mix
scripts/hardware/servo_test.c's pre-existing SPDX issue into this reconciliation.
