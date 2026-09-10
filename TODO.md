# NOMAD work ledger

One ledger; one active item. Requirements/decisions live in [PRD](docs/prd.md),
implementation/evidence in [migration](docs/migration.md), and transport details
in [MAVSDK adoption](docs/mavsdk-adoption.md).

Status: [ ] open, [~] active, [x] complete. A task being implemented does not
close its integration or release gate.

## Current work

- [~] G-M Phase A resource qualification: the published MAVSDK graph now has
  recursive clean-checkout provenance/build evidence, hosted Linux/Windows/ROS
  qualification and a live ArduPilot Copter SITL connect/status pass. Collect
  repeatable build/runtime measurements and obtain explicit approval for build
  tree, executable, memory, startup and CI-time budgets before closing Phase A.
  Falsification: the reviewed pins stop reproducing, live qualification regresses,
  or measured resources exceed an approved threshold.

## Ordered implementation backlog

- [ ] G0: obtain organizer rulings Q02-Q09 and assign named decision/gate owners
  (D10). The 125-entry documentation reconciliation and Q01 decision are complete;
  unanswered interpretations keep the planning gate open.
- [ ] G-M Phase B connection boundary: introduce the narrow MAVSDK connection
  owner behind the existing core boundary without switching production or
  duplicating vehicle policy. Requirements: project MAVSDK decision, C01 and
  GAP-16. Falsification: boundary tests accept an invalid/wrong peer, conceal
  loss, or alter the default legacy production path.
- [ ] G-M Phases B–D: MAVSDK command, telemetry, velocity/watchdog, GCS heartbeat,
  zero-delivery, fence and parameter parity; unit plus integration tests.
- [ ] G-M: add ArduPlane/QuadPlane support coverage for Task 1 alongside Copter;
  prepare focused reviewable implementation changes with independent evidence.
- [ ] G-M Phase E: switch production to MAVSDK and remove the old implementation
  only after parity, traceability, profile and release checks pass.
- [ ] G-M Phase F: upstream tested ArduPilot fixes; track patches, review status
  and maintenance owner. Upstream acceptance is separate from local parity.
- [ ] G2 / C04–C09, C13, C15: single persistent command owner, authenticated client
  contract, fresh telemetry, aircraft-specific semantics, cancellation and
  truthful final outcomes; migrate direct plugin policy and hazardous outputs.
- [ ] G3: runtime capability and profile tests; Task 1 GPU-groundstation video/CV
  path, Task 2 optional onboard path, minimal operation without perception.
- [ ] G4: official armed telemetry and traffic cylinders after Q04; independent timing
  mock, demonstrated operator avoidance, outage/replay and official-server acceptance.
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

- [x] C++ library/CLI, generated MAVLink UDP transport, telemetry and basic
  Copter vehicle/mission operations exist.
- [x] Velocity configuration, watchdog, VIO source validation, fence and
  dedicated payload safety paths with unit tests exist.
- [x] Wire-level zero-delivery tests and named SITL harnesses exist; live
  zero-delivery and containment gate evidence remains open.
- [x] Mission Planner core client for goto/discrete outputs and ROS adapter exist;
  full command ownership migration remains open.
- [x] MAVSDK optional Phase A build/smoke target and fork wiring exist;
  production migration is still open.
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
