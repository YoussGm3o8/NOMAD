# NOMAD work ledger

One ledger; one active item. Requirements/decisions live in [PRD](docs/prd.md),
implementation/evidence in [migration](docs/migration.md), and transport details
in [MAVSDK adoption](docs/mavsdk-adoption.md).

Status: [ ] open, [~] active, [x] complete. A task being implemented does not
close its integration or release gate.

## Current work

- [~] G1: run live image/SITL/ROS and clean-boot service qualification on the
  repaired build, profile, service and setup paths. C02 and C10-C12 source
  repairs are recorded in migration.

## Ordered implementation backlog

- [ ] G0: reconcile full CONOPS and assign remaining decision owners. The planning
  baseline and user hardware/autonomy/MAVSDK direction are recorded.
- [ ] G-M Phase A: hosted Linux/Windows/ROS builds, live MAVSDK connect/status,
  transitive notices and approved dependency footprint.
- [ ] G-M Phases B–D: MAVSDK command, telemetry, velocity/watchdog, GCS heartbeat,
  zero-delivery, fence and parameter parity; unit plus integration tests.
- [ ] G-M: add ArduPlane/QuadPlane support coverage for Task 1 alongside Copter;
  prepare focused implementation merge request(s) with independent evidence.
- [ ] G-M Phase E: switch production to MAVSDK and remove the old implementation
  only after parity, traceability, profile and release checks pass.
- [ ] G-M Phase F: upstream tested ArduPilot fixes; track patches, review status
  and maintenance owner. Upstream acceptance is separate from local parity.
- [ ] G2 / C04–C09, C13, C15: single persistent command owner, authenticated client
  contract, fresh telemetry, aircraft-specific semantics, cancellation and
  truthful final outcomes; migrate direct plugin policy and hazardous outputs.
- [ ] G3: runtime capability and profile tests; Task 1 GPU-groundstation video/CV
  path, Task 2 optional onboard path, minimal operation without perception.
- [ ] G4: competition telemetry/events and traffic adapter; independent mock,
  advisory deconfliction, outage/replay tests and official-server acceptance.
- [ ] G5: VTOL survey plan/execution, detection/count/identity evidence, simulated
  cooperation, reserve and safe landing.
- [ ] G6: custom tracker/tagging/sampling design integration, path mapping,
  explicit authorization and Task 2 battery-swap recovery.
- [ ] G7: authorized hardware evidence for selected controller, GNSS/RTK, camera,
  RF paths, all-up mass/endurance, payload feedback and safe failure behavior.
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
- [x] Three profile templates/config tests and retained Python video bridge exist;
  runtime and hardware capability evidence remains open.
- [x] Documentation review baseline: test-core 9/9; test-python 251 passed,
  3 skipped on 2026-09-08. See migration for subsequent quality checks.

## User decisions recorded on 2026-09-08

- Task 1: proposed lightweight VTOL, ground CV, no Jetson onboard; Pi Zero LTE
  backup and possible video transport.
- Task 2: heavier quad under the preview mass limit; optional Jetson if needed.
- Walksnail FPV camera; second camera TBD; no reliance on ZED.
- Flight-controller IMU; Cube Orange or custom ArduPilot controller; Here4 and
  RTK base selected in principle. Actual integrations remain unqualified.
- Custom tracker, possibly ESP32; payload mechanisms and feedback TBD.
- Traffic advisories and explicit action authorization first; revisit autonomy
  after CONOPS.
- MAVSDK is mandatory for competition and an early work priority with tests and
  a merge request. This pass produces the plan, not that implementation.

## Decisions still open

See D01–D11 in PRD: exact aircraft/firmware and profiles, core host/IPC/remote
transport, capture and synchronized sensor access, tag/sample design, primary
radio, numeric performance/safety budgets, server contract, owners and evidence
retention. The "15 kg class" description does not override the below-15-kg rule.
