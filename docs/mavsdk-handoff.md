# MAVSDK compatibility closeout

Updated 2026-09-19. This record follows the implementation handoff from NOMAD
PRs [#9](https://github.com/YoussGm3o8/NOMAD/pull/9),
[#10](https://github.com/YoussGm3o8/NOMAD/pull/10), and
[#11](https://github.com/YoussGm3o8/NOMAD/pull/11). It does not approve an aircraft
release. The canonical [migration plan](migration.md),
[adoption decision](mavsdk-adoption.md), and
[dependency inventory](mavsdk-dependencies.md) retain their respective authority.

## Rebased fork

`YoussGm3o8/MAVSDK`, branch `nomad/ardupilot`, publishes qualified revision
`3f85f6f808b617c736316d7da5f51f3d3eba1737`, rebased onto upstream main
`d7043d3cafe8cd6250565fd211b966d8b455d561`. The previous consumer pin was
`e0dada26a606ffa4f48e72841efa231177733d05`, retained on the published
`ardupilot-before-rebase-20260919` branch for existing checkouts and rollback.
The fork's independent build,
pinned ArduCopter 4.7.1 inputs and exact executable compatibility matrix live in
`docs/en/cpp/guide/ardupilot_compatibility.md` in that repository.

## Completed implementation

- Debian 11's live security indexes referenced unavailable OpenSSL and ICU
  packages even after refresh. The retained packaging job now uses signed dated
  Debian snapshots and requires index refresh to succeed. This is a reproducible
  build environment, not a claim of continuing Debian 11 security support.
- iOS applied an overlapping MAVLink patch after the common patch. Every platform
  now applies the pinned-generator patch once. Device, simulator and XCFramework
  jobs remain enabled. NOMAD's ROS, Jetson and Isaac Dockerfiles normalize only
  the retained patch; local MAVSDK build caches are excluded from image inputs.
- Windows combined ZIP uploads used repository-root paths even though ZIPs were
  created under `cpp/`. Upload and release paths now name the actual directory,
  and a missing combined archive fails the workflow instead of only warning.
- `ArduPilotFenceFaults.ReportsVehicleRejectedUploadAndRecovers` independently
  observes `MAV_MISSION_NO_SPACE` and the SDK rejection result before recovery.
  `UploadTimesOutDuringResponseLossAndRecovers` and
  `DownloadTimesOutDuringResponseLossAndRecovers` drop actual vehicle replies,
  count the loss, bound completion, restore the link path and verify recovery.
  `RoundTripsExclusionPolygon` extends the existing inclusion-fence evidence.
- `Offboard::set_velocity_body_once` queues one body-velocity frame without
  storing a setpoint, changing mode or starting automatic resends. Five
  `OffboardOneShot` wire tests cover stalled producers, invalid inputs, zero and
  destruction, removed connections and refusal to mix automatic resend modes.
  `OneShotVelocityExpiresWhenProducerStalls` verifies movement followed by expiry
  under the vehicle's unchanged `GUID_TIMEOUT`.
- NOMAD now calls that API and converts yaw rate from radians to degrees. MAVSDK
  owns the frame, mask and packing. NOMAD retains authorization, freshness,
  watchdogs, velocity limits, cancellation, final zeros and authoritative checks.
  Independent consumer wire tests and live stop probes qualify the transition.
  Both hosted Linux/Windows consumer jobs now run the full Phase B fixture.
- Four `OperationDeadlineWire` tests cover lost command/parameter attempts
  followed by success, repeated progress acknowledgements and concurrent
  independent deadlines. Upward rounding to timer precision prevents spurious
  retries just before expiry; `OperationTimeout.FinalAttemptNeverRoundsBeforeTheDeadline`
  covers the boundary. Existing overloads retain their SDK-wide timeout behavior.
- Rebased SDK system IDs are checked as 32-bit values before narrowing. NOMAD's
  identity regression rejects 257 when the expected MAVLink ID is 1.
- Three isolated upstream contributions are open:
  [pinned generator and shared iOS patch](https://github.com/mavlink/MAVSDK/pull/3102),
  [geofence input validation](https://github.com/mavlink/MAVSDK/pull/3103),
  and [Windows archive paths](https://github.com/mavlink/MAVSDK/pull/3104).
  The first two focused regressions pass and fail against the original implementations.
  All three Windows archives are independently verified as published and nonempty;
  the downloaded x64 ZIP contains both configurations, headers and CMake metadata.
  Submission is not upstream acceptance; retain patches until merged and requalified.

## Exact-revision evidence

| Check | Result and evidence |
|---|---|
| Standalone ArduCopter compatibility | Passed, all 33 tests: [run 35481870540](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870540) |
| Fork Linux matrix | Passed, including Debian 11 build, unit/system tests and installation: [run 35481870573](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870573) |
| Fork Apple matrix | Passed: all four macOS builds, three iOS builds and XCFramework: [run 35481870483](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870483) |
| Fork Windows matrix | Passed, all 14 jobs and three combined archives: [run 35481870500](https://github.com/YoussGm3o8/MAVSDK/actions/runs/35481870500) |
| Local fork suites (unchanged SDK source) | Passed: 415 unit tests, 114 system tests, 33 fresh pinned-Copter compatibility tests |
| Linux deadline stability | Passed: 20 repetitions of all four wire tests, 80 checks |
| NOMAD core and wire fixtures | Passed: 9 CTests; Phase A valid/wrong/absent peer; Phase B command, deadline, link, fence and all five zero-delivery cases |
| NOMAD live stop behavior | Passed: `core_sitl_zero_delivery.py` observed the final zero, hover and disarm; `core_sitl_velocity_watchdog.py` observed `command_timeout` and disarm |
| NOMAD ROS adapter | Passed: fresh `Dockerfile.sim-ros` image build and all 9 ROS integration tests |
| Consumer pin/provenance and Phase A | Passed: reviewed gitlinks, dependency/license checks, 16 tests and valid/wrong/absent peer fixture |
| Python and quality checks | Passed: 514 Python tests, 3 environment skips; lint, format check and strict documentation build |

The complete fork platform workflows passed before publication of this consumer
update. The gitlink and provenance records adopt that qualified revision together.
The dedicated Copter result qualifies the named surface, not every plugin,
platform or vehicle.
Linux completed 30 jobs successfully; its conditional `Update Docker Images`
job was skipped. Its style and API regeneration check passed. The Apple
XCFramework/framework artifacts and all three combined Windows ZIPs
are published and nonempty. Tag-only Debian package creation and release-upload
steps were not executed; no tagged release is qualified by these branch runs.

The preceding revision `f1381975b403f497ca855187d9e8c9d19a22fb99` passed
Linux, Apple, all 33 hosted compatibility tests and Windows on retry. Its Windows
upload warnings exposed the archive-path fix above; SDK source and dependency
pins are identical between that revision and the final pin.

`SystemTest.RequestMessageRapid` and its request implementation are unchanged
from upstream. The failing Windows test passed 20 consecutive local repetitions
and another 100 with command diagnostics enabled. Its hosted rerun passed;
the final pin's complete Windows matrix passed on its first attempt.
This does not establish that
the intermittent timeout's cause has been fixed.

debt: one hosted rapid-request timeout has not reproduced in 120 local runs;
the fork maintainer revisits on another hosted occurrence, then captures the
request/ACK lifecycle and adds a deterministic regression before changing it.

## Remaining release gates and maintenance

Calibration, the high-level Mission API and legacy telemetry-rate fallback remain
**Unknown**. Cover them when a required consumer/version needs them. Aircraft,
vehicle-class/QuadPlane qualification, resource budgets, supported-aircraft policy,
and packaging/rollback evidence remain under the existing migration gates.
The proto source has no separate license declaration in its current tree; it is
not built by NOMAD, and enabling the server requires a new dependency audit.

General commands and heartbeat observation retain a justified thin passthrough
adapter; their application policy belongs to NOMAD. The ownership audit names
the exact boundaries and replacement triggers. Do not infer flight containment
from successful fence transfer or vehicle outcomes from command acknowledgements.

For every subsequent pin, publish and qualify the fork first, then update the
gitlink, provenance checker, adoption record and dependency inventory together.
Run consumer and safety checks on that exact pin:

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

Original compatibility issue closure still requires executable evidence for its
required surface, justified owners for remaining adapter paths and a qualified
consumer pin. Unknown release entries remain Unknown.
