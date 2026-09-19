# MAVSDK compatibility handoff

Recorded 2026-09-19. This document closes the current implementation task and
records remaining work; it does not close every acceptance criterion of the
original compatibility issue or approve an aircraft release. The canonical
[migration plan](migration.md), [adoption decision](mavsdk-adoption.md), and
[dependency inventory](mavsdk-dependencies.md) retain their respective authority.

## Delivered baseline

- NOMAD implementation PRs [#9](https://github.com/YoussGm3o8/NOMAD/pull/9),
  [#10](https://github.com/YoussGm3o8/NOMAD/pull/10), and
  [#11](https://github.com/YoussGm3o8/NOMAD/pull/11) are merged.
- NOMAD baseline merge: `aa55c7ef0f859bf72f7da74ef643dd529b19a813`.
- Fork: `YoussGm3o8/MAVSDK`, branch `nomad/ardupilot`, pinned by NOMAD at
  `e0dada26a606ffa4f48e72841efa231177733d05`.
- The fork documents its purpose, upstream tracking, pinned ArduCopter 4.7.1
  inputs, standalone build, test procedure, and executable compatibility matrix
  in `docs/en/cpp/guide/ardupilot_compatibility.md`.
- Generic operation deadlines cover command transport and typed parameter reads,
  including retries and queue time. NOMAD uses `OperationOptions` without
  changing the SDK-wide timeout or knowing its retry count.
- Relative goto uses `Action::goto_location_relative`; MAVSDK owns the Guided
  command representation, relative-altitude frame, and coordinate encoding.
- MAVSDK initializes core ArduPilot telemetry with message interval requests.
  NOMAD's unused stream-request surface was removed.
- Fence transfer uses MAVSDK Geofence. NOMAD retains fence policy, identity,
  authorization, freshness, watchdogs, velocity limits, and authoritative checks.
- Marker-based API regeneration preserves compatibility overloads. Hosted
  generator/style checks passed for the pin.

## Verification at the pinned revision

| Check | Result and evidence |
|---|---|
| Standalone ArduCopter compatibility | Passed: [run 35421014588](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35421014588), 19 compatibility tests |
| Fork Windows matrix | Passed: [run 35421014680](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35421014680) |
| Fork Linux matrix | Failed only in Debian 11 packaging: [run 35421014713](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35421014713); other jobs passed or were intentionally skipped |
| Fork Apple matrix | All four macOS desktop jobs passed; three iOS jobs failed and XCFramework was skipped: [run 35421014584](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35421014584) |
| NOMAD PR #11 checks | Passed before handoff, including core, Python, Linux/Windows MAVSDK consumers, ROS integrations, and lint |
| Local consumer checks | Provenance, Phase A fixture, and Phase B command/link/fence/velocity fixture passed during closeout |
| Handoff documentation | Strict documentation build and diff whitespace check passed |

The normal fork matrix is therefore not wholly green. The dedicated SITL result
qualifies the documented Copter surface, not all platforms, plugins, or vehicles.

## Remaining work, in priority order

1. **Fork maintainer: repair the two hosted build failures.** Debian 11 logs show
   HTTP 404 responses downloading OpenSSL and ICU packages. Inspect the container
   package indexes and refresh them before installation, then rerun that job.
   All three iOS jobs fail applying the MAVLink dependency patch to
   `CMakeLists.txt:18`; reproduce the patch sequence from a clean iOS dependency
   checkout, resolve patch ordering/context, and rerun device and simulator jobs.
   Do not suppress these jobs or claim success based on desktop builds. Exit:
   complete Linux and Apple workflows pass on the proposed fork commit.
2. **Fork maintainer: close remaining geofence evidence gaps.** Vehicle-side
   rejected upload and timeout against actual ArduCopter remain **Unknown**.
   Invalid input rejection and synthetic silent-peer download timeout are tested.
   Add independent wire/result assertions for vehicle rejection and a controlled
   link-loss test for upload/download timeout, including recovery. Extend polygon
   round-trip evidence to exclusion type if claiming that support. Exit: pinned
   SITL regressions pass and the matrix names their exact tests.
3. **Fork and NOMAD maintainers: resolve the one-shot velocity decision.** The
   fork proves Offboard forward/right/down/yaw-rate movement, repeated setpoints,
   zero, and stop. NOMAD still packs one-shot body setpoints through passthrough
   because automatic resend could retain a stale command if its watchdog stalls.
   Keep the documented contract until a generic one-shot/expiring API or measured
   fault-injection evidence proves equivalent behavior. Test stalled producers,
   cancellation, shutdown, and lost links before replacing this path. Exit:
   independently tested generic transport plus NOMAD stop/freshness regressions;
   retain all application policy above it.
4. **Fork maintainer: strengthen deadline and retry qualification.** Existing
   deterministic `OperationTimeout` tests and queued/silent SITL cases establish
   the current budget behavior. Add controlled loss followed by a successful
   retry, repeated IN_PROGRESS acknowledgements, and concurrent independent
   deadlines when extending this API. Verify wire attempts and bounded completion
   without changing the SDK-wide timeout. Preserve existing overload behavior.
5. **Fork maintainer: prepare upstream contributions.** No upstream PR is claimed
   by this task. Split the compatibility changes into reviewable units with
   isolated regressions, relate them to the upstream issues linked in the fork
   guide, and submit where practical. Rebase and requalify before dropping patches.
6. **NOMAD maintainer: requalify every subsequent pin.** Publish and qualify the
   fork first, then update the gitlink, provenance checker, adoption record, and
   dependency inventory together. Run consumer and safety checks on the exact pin.
   Keep resource budgets, vehicle-class/QuadPlane qualification, supported-aircraft
   policy, and packaging/rollback evidence under the existing migration gates.

Calibration, the high-level Mission API, and legacy telemetry-rate fallback also
remain **Unknown**. Add coverage when a required consumer/version needs them;
do not infer support from the successful Copter tests.

## Resume and closure checks

Start with the failed job logs linked above and the fork's compatibility guide.
Use a focused branch for each fix, preserve the ownership audit in the adoption
record, and run the fork independently of NOMAD. Consumer checks are:

```sh
pixi run check-mavsdk-phase-a
pixi run test-mavsdk-phase-a
pixi run test-mavsdk-phase-b
pixi run test-core
pixi run test-python
pixi run lint
pixi run docs-build
```

The original issue can be closed only after its required compatibility surface
has executable evidence, the remaining adapter paths have a justified owner,
and the exact consumer pin is qualified. This handoff records the remaining
exceptions explicitly; it does not turn Unknown entries into Tested entries.
