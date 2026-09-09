# MAVSDK adoption decision and parity gates

Decision originally recorded 2026-09-06; confirmed by the user on 2026-09-08:
MAVSDK ArduPilot support must be used during competition. Adoption is an early
implementation priority with unit tests, integration tests and a focused merge
request. It is gate G-M in [migration](migration.md), a prerequisite for release.

This is the transport decision record. Product phases, hardware and competition
requirements live in the canonical migration, architecture and PRD documents.

## Current status

The production core still uses UdpMavlinkConnection and generated ArduPilot
dialect headers. CMake NOMAD_ENABLE_MAVSDK defaults OFF; enabling it builds a
separate connect/status smoke executable, not a MAVSDK-backed Vehicle.

Phase A source exists: opt-in subbuild, telemetry/passthrough smoke consumer,
scripts/dev/mavsdk_phase_a_smoke.py, Linux/Windows CI jobs, optional ROS image
wiring, root NOTICE and a fork submodule/branch. Read .gitmodules and the gitlink
for actual provenance; no new fork or remote change is required by this plan.
Phases B–E production parity remain open.

Historical local notes reported Windows Release and Linux-in-ROS builds, but
the later Windows smoke failed closed without sustained telemetry. An earlier
evaluation reported a successful stream; that is a different artifact/run and
does not close current Phase A. Hosted build/SITL evidence and the transitive
dependency notice audit remain open.

## Ownership and rationale

MAVSDK supplies wire protocol, transport and its internal workers. NOMAD retains
Vehicle, safety policy, mission decisions, target identity, authoritative
verification, application deadlines, audit and client contract. Internal library
threads do not by themselves prove bounded watchdog behavior.

The adoption supersedes the 2026-09-05 own-codec decision before its original
revisit triggers (new vehicle ecosystem, direct serial/TCP need, upstream quirks)
were met. The recorded rationale was maintained transport ownership and upstream
contribution. The user now explicitly makes adoption a competition deliverable.

Keep the fork patch set small, pin reviewed dependencies, record firmware/library
pairs and rerun qualification on every pin change. The old firmware-matched
dialect property must be replaced by tested compatibility with both Copter and
the chosen ArduPlane/QuadPlane firmware. The library API/quirks below must be
verified against the pinned source; they are not claims about latest upstream.

## Phase A — Build, dependencies and telemetry

Keep the opt-in build isolated while measuring footprint and reviewing notices.
The current setup selects static libraries, telemetry, passthrough, no gRPC
server, no MAVSDK tests, and BUILD_WITHOUT_CURL=ON. Revisit options only for a
demonstrated required plugin.

Exit: reproducible Linux/Windows/selected ROS builds; live connect/status with
correct ownship, mode and scaled telemetry; no-peer and wrong-peer failures;
dependency notice audit and approved build/runtime resource budgets.

Historical Windows measurements (2026-09-06, MSVC 19.44, Release, pinned v3
checkout) are retained as comparison data, not acceptance thresholds:

| Measurement | Recorded value |
|---|---|
| Clean configure including dependency builds | 3m14.806s |
| Following clean target build | 3m8.493s |
| Warm optional build | 2m57.515s |
| Legacy CLI executable | 148,480 bytes |
| Optional MAVSDK smoke executable | 2,020,352 bytes |
| Selected static archives total | 118,453,210 bytes |
| Optional build tree / legacy tree | 446,770,697 / 13,890,039 bytes |

The executable comparison is not a like-for-like product size comparison.
Approve actual latency/memory/binary/CI budgets before deciding Phase A passes.

## Phase B — Vehicle and output parity

Implement the existing connection boundary using only required MAVSDK APIs.
Keep public safety/Vehicle semantics, CLI spelling, client errors and argument
validation stable unless a reviewed correction changes an unsafe contract.

Cover arm/disarm, mode, takeoff, land/RTL, goto, servo, relay, motor-test,
gimbal-config and user-command. Evaluate passthrough or a narrow fork extension
for unsupported commands. Verify COMMAND_INT location semantics and altitude
datum rather than assuming a generic goto call is equivalent.

Add explicit vehicle-class identification and reject incompatible operations.
Task 1 requires Plane/QuadPlane coverage, not a renamed Copter test. Pin mode
mapping, transition state and accepted command matrix.

Exit: fake-transport and codec-independent contract tests, CLI/plugin tests and
Copter command-flow/payload SITL pass; negative ACK, wrong target, delayed state,
timeout and cancellation results remain truthful. QuadPlane-specific mission/
transition tests qualify new operations before G2/G5, not merely discovery.

## Phase C — Velocity, watchdog, stop and heartbeat parity

Evaluate Offboard start/stop and any automatic background resend against
ArduPilot GUIDED semantics. A narrow raw-setpoint path is an option if needed,
not permission to bypass NOMAD gates. Stop must end stale resends; destruction,
reconnect and callback lifetime must not revive control.

Exit: each watchdog fault case; independent wire zero-delivery plus
core-sitl-zero-delivery; core-sitl-velocity-watchdog; GCS-heartbeat relay gate
with its dropped-announcement negative control. Verify identities, cadence,
thread contention and shutdown order. Fixed-wing safety is separately defined;
do not apply the Copter velocity scenario as a QuadPlane flight guarantee.

## Phase D — Fence, parameters and mission integration

Verify actual fence mission items, vertex semantics, readback and enable state
using Geofence/Param or narrowly required protocol access. Keep malformed input
and disabled/unreadable fence failure tests.

Exit: core-sitl-geofence and sitl-fence containment evidence; full relevant
parameter/fence state matches the reviewed plan. Add any mission upload/progress/
abort APIs required by Task 1 in a focused extension with independent QuadPlane
SITL evidence, not an untested side effect of transport replacement.

## Phase E — Production cutover

Switch the production connection after A–D pass. Retain golden/wire semantic
references until equivalent coverage survives replacement. Remove obsolete
codec/UDP/generation code and possibly its submodule only after caller inventory,
profile checks and stable safety traceability remapping.

Exit: default CLI/runtime and clients demonstrably use MAVSDK; full unit,
adapter and SITL matrix passes for supported firmware; dependency notices and
packaging verified. The old path cannot be a hidden runtime fallback. Competition
release G8 cannot pass with MAVSDK limited to a smoke executable.

## Phase F — Upstream work and maintenance

Prepare focused merge requests/PRs with reproductions for confirmed ArduPilot
issues: GUIDED mode interpretation, battery units, location command semantics,
and any stop/heartbeat/fence/QuadPlane fixes. Recheck each historical finding
against pinned/upstream source before patching.

Required local fixes must have tests and an accountable maintainer even if
upstream review is pending. The implementation merge request should contain
problem/behavior, requirements, exact pins, unit/integration evidence and residual
limitations. This planning change neither implements nor publishes it.

debt: only required ArduPilot fork patches; revisit on each upstream release or
quarterly maintenance review; then upstream/drop resolved patches and requalify
the firmware/library matrix. Numeric patch/footprint ceilings require D08/D10;
no unspecified threshold is treated as a passed gate.

## Risks to keep visible

- Library/firmware dialect drift: pinned pairs and independent wire/SITL tests.
- Mode and Offboard semantic mismatch: aircraft-specific validation and stop tests.
- Background resend/concurrency: prove no stale motion after cancel/shutdown.
- Plugin/API availability: inspect pinned source before choosing a wrapper.
- Build size and transitive notices: measurable Phase A acceptance, not optimism.
- Schedule: adoption is mandatory; prioritize it before integrated competition
  commands, while isolated perception/server test fixtures can progress.
