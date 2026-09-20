# NOMAD / MAVSDK project handoff — 2026-09-20

This snapshot records the state immediately after NOMAD PR #13 was reviewed and merged. It is a project handoff, not flight approval. Canonical requirements remain in `docs/prd.md`, architecture in `docs/architecture.md`, implementation/release evidence in `docs/migration.md`, and the actionable ledger in `TODO.md`.

## Baseline

- NOMAD `main`: `26d7f9b101a029725d06aee2c6716da95e622417` (merge of PR #13).
- NOMAD now pins `YoussGm3o8/MAVSDK:nomad/ardupilot` at `3f85f6f808b617c736316d7da5f51f3d3eba1737`.
- The fork pin is rebased on upstream MAVSDK `d7043d3cafe8cd6250565fd211b966d8b455d561`.
- Rollback/reference pin: `e0dada26a606ffa4f48e72841efa231177733d05`, retained on `ardupilot-before-rebase-20260919`.
- The stale NOMAD branch `feature/audio-altitude-callouts` is 75 commits behind `main`; its substantive altitude-callout work is already represented in later history. Do not merge that branch as-is.

## What PR #13 changed

PR #13 moves the final NOMAD-owned velocity wire encoding into the reviewed MAVSDK fork while keeping NOMAD's safety policy above it.

- `Offboard::set_velocity_body_once` sends exactly one body-frame velocity setpoint and does not start MAVSDK's automatic resend loop.
- NOMAD retains command authorization, velocity limits, VIO/freshness gates, watchdog behavior, cancellation/shutdown handling, and final zero setpoints.
- The fork owns the MAVLink frame, type mask, field encoding and yaw-rate units. NOMAD converts its rad/s value to the MAVSDK deg/s API.
- Wide SDK system IDs are validated before narrowing, so values such as 257 cannot alias MAVLink system ID 1.
- Linux and Windows NOMAD CI now run the full Phase B command/link/fence/velocity fixture.
- Docker image inputs were reconciled with the rebased fork: the removed iOS MAVLink patch is no longer normalized, and local MAVSDK build caches are excluded.
- Provenance, dependency inventory and MAVSDK ownership documentation now point at the exact qualified pin.

The review found no blocking correctness or safety regression in the change. The exact PR head passed NOMAD `test`, `lint` and `ros-sim` hosted workflows before merge.

## MAVSDK fork qualification

The exact pinned fork revision completed the currently relevant hosted qualification:

- Linux workflow: passed, including Debian 11 packaging/build coverage.
- Apple workflow: passed, including macOS, iOS and XCFramework jobs.
- Windows workflow: passed, including all combined archive jobs.
- ArduPilot compatibility workflow: passed all 33 pinned ArduCopter compatibility tests.
- One-shot velocity has independent wire tests for frame/mask/units, no stale resend, zero/destruction, invalid input, removed links and refusal to mix with automatic Offboard resends.
- Pinned ArduCopter SITL proves a one-shot Guided velocity command expires under the unchanged vehicle `GUID_TIMEOUT` when the producer stalls.
- Deadline regressions cover a lost first command followed by success, repeated `IN_PROGRESS` acknowledgements, independent concurrent deadlines, and a lost first parameter read followed by success.
- Fence regressions cover malformed input, vehicle rejection, transfer loss/recovery and exclusion polygons in the fork qualification set.

One previously observed hosted Windows rapid-request timeout did not reproduce in repeated local runs and the final Windows matrix passed. Treat another occurrence as evidence to investigate, not as a closed root cause.

## Upstream MAVSDK pull requests

Three isolated contributions are open against `mavlink/MAVSDK`:

1. `#3102` — **build: use the pinned MAVLink generator on desktop and iOS**. It removes build-time pip upgrades, uses the pinned pymavlink submodule and replaces the overlapping desktop/iOS patch sequence with one reproducible patch. The change is focused and has a no-pip generator regression. Keep it open for upstream review; upstream Actions currently require repository-side approval/action for fork workflows, so those statuses are not evidence of a source regression.
2. `#3103` — **geofence: validate geometry before starting mission transfer**. It rejects short polygons, invalid coordinates, invalid circles and wire-count overflow before emitting `MISSION_COUNT`, with hardware-free system tests. The review found no blocker.
3. `#3104` — **ci: upload Windows archives from their actual directory**. It fixes the demonstrated `cpp/` path mismatch for artifacts/releases and makes missing combined archives fatal. The downstream full Windows run proves the corrected paths produce non-empty archives. Tag-triggered release publication itself has not been exercised.

Do not remove equivalent fork patches merely because an upstream PR is open. After an upstream merge, rebase/re-pin deliberately and rerun the exact consumer qualification before deleting a local patch.

## Immediate next action

Run the **full hosted Copter SITL matrix on the merged `main` commit** before starting the next feature slice. PR #13 changed the production velocity transport and the pinned MAVSDK graph; its ordinary hosted CI and focused/live stop probes passed, but the complete hosted SITL matrix should be re-established on this exact post-merge baseline.

Exit condition: telemetry smoke, connection/status, command flow, mission execution, velocity watchdog, zero delivery, payload relay, link loss/recovery, GCS-heartbeat gate, velocity loop closure, geofence containment and geofence upload/readback all pass on the current pin, and the run URL is recorded in the migration evidence.

## Work after current-head SITL

Proceed in this order:

1. **Reconcile the canonical status documents.** `TODO.md` still marks the GCS-heartbeat evidence active even though that slice has previously passed, and top-level status wording should reflect that MAVSDK is the sole production transport. Update status only against recorded current-head evidence.
2. **Close or explicitly defer the Phase A resource-budget gate.** The repository already collects build-tree size, executable size, process-tree RSS, startup time and CI duration. Record approved numerical limits/owners and compare the retained artifacts against them; do not leave “budget approval” permanently qualitative.
3. **Add explicit aircraft-class support and qualify ArduPlane/QuadPlane.** This is the next substantive NOMAD implementation slice. Carry the autopilot/vehicle type from heartbeat discovery into the core state or an equivalent validated identity boundary; reject unsupported/unknown aircraft-specific behavior; remove Copter-only mode assumptions from generic vehicle outcomes; then add focused Plane/QuadPlane SITL coverage for the Task 1 VTOL path. Preserve the existing Copter matrix as regression coverage.
4. **Complete G-M release evidence for the supported aircraft set.** Re-run the current-head ROS/SITL matrix for every declared vehicle class and add reproducible packaging/install/rollback evidence.
5. **Then move into G2 single-command ownership.** Introduce the persistent C++ command owner and versioned client contract; make Mission Planner, CLI and ROS clients of that owner in integrated mode; serialize incompatible requests; add cancellation/session semantics and truthful admitted/transmitted/ACK/state-verified/cancelled/unknown outcomes. Migrate direct Mission Planner safety/output policy or confine it to documented maintenance paths.

Only after these predecessor gates should G3 profile qualification and the G4 AEAC server/traffic module become the main integration thread. Isolated CV/server mocks may continue in parallel if they do not create another vehicle command owner.

## MAVSDK work priority

Do not expand the fork merely for completeness. The current pin covers NOMAD's required Copter surface well enough to move the project forward. Near-term MAVSDK work should be maintenance-driven:

- respond to upstream review on #3102–#3104;
- retain the qualified pin unless NOMAD needs a concrete new capability or an upstream change must be consumed;
- when advancing the pin, publish/qualify the fork first, then update the gitlink, provenance checker, adoption record and dependency inventory together;
- rerun Phase A, Phase B, core, Python, lint/format/docs, ROS and applicable SITL evidence on the exact new pin;
- consider upstreaming the remaining generic pieces (operation deadlines, relative-altitude goto, one-shot body velocity and ArduPilot telemetry semantics) only as small independently falsifiable contributions, without blocking NOMAD's QuadPlane/G2 work.

The MAVSDK proto subtree currently has no separately established license declaration in the NOMAD dependency audit. It is not built into the selected NOMAD server-disabled configuration. Re-audit before enabling/distributing the server path.

## Known release blockers

The current software baseline is not an aircraft release. Remaining blockers include current-head full SITL evidence after the new pin, explicit resource budgets, supported-aircraft/QuadPlane qualification, packaging/install/rollback, the single integrated command owner, hardware qualification, competition-specific server/traffic semantics, and the G7/G8 flight/rehearsal evidence.

## Resume checks

For a new MAVSDK pin or before claiming the compatibility baseline remains intact, run:

```sh
pixi run check-mavsdk-phase-a
pixi run test-mavsdk-phase-a
pixi run test-mavsdk-phase-b
pixi run test-core
pixi run test-python
pixi run lint
pixi run format-check
pixi run docs-build
```

Then run the applicable ROS and hosted SITL workflows. Passing unit/fixture tests alone is not evidence for aircraft-class, containment, packaging or flight readiness.
