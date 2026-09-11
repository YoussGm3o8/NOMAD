# MAVSDK adoption decision and parity gates

Decision originally recorded 2026-09-06; confirmed by the user on 2026-09-08:
MAVSDK ArduPilot support must be used during competition. Adoption is an early
implementation priority with unit tests, integration tests and focused reviewable
changes. It is gate G-M in [migration](migration.md), a prerequisite for release.

This is the transport decision record. Product phases, hardware and competition
requirements live in the canonical migration, architecture and PRD documents.
CONOPS v1.0 does not mandate MAVSDK; mandatory adoption is the project decision.
Its [source requirements](conops-requirements.md) define the acceptance context:
100 m AGL, all-mode termination, traffic separation and task evidence.

## Current status

The production core still uses UdpMavlinkConnection and generated ArduPilot
dialect headers. CMake NOMAD_ENABLE_MAVSDK defaults OFF; enabling it builds a
separate connect/status smoke executable, not a MAVSDK-backed Vehicle.

Phase A source exists: opt-in subbuild, telemetry smoke consumer, deterministic
ArduPilot-like UDP fixture, pure qualification tests, Linux/Windows CI jobs,
selected ROS image compile wiring, dependency inventory, root NOTICE and a pinned
project MAVSDK fork. The parent gitlink now pins
`9884f109533f564bc6250e5471e6301d3a62f4a7`; read `.gitmodules`, the gitlinks and
the [dependency inventory](mavsdk-dependencies.md) for provenance. Phases B-E
production parity remain open.

The published Phase A graph has now passed recursive hosted qualification. Test
run `34535620056` completed the Python suite, C++ core, provenance checker,
deterministic peer fixture and optional MAVSDK build on both Ubuntu and Windows.
Selected ROS-image run `34538394497` built with NOMAD_ENABLE_MAVSDK=ON and passed
the ROS adapter integration suite. Mainline SITL run `34538903820` built the
ArduPilot Copter 4.7.1 image, started the development/SITL stack and passed the
MAVSDK Phase A connect/status smoke. Subsequent changes through the mainline
workflow cleanup did not alter the selected MAVSDK source graph. These runs close
the clean-checkout/hosted/live-Copter evidence portion of Phase A; they do not
qualify command parity, aircraft hardware or QuadPlane behavior.

The remaining Phase A release blocker is resource qualification: collect
repeatable build-tree, executable, memory, startup and CI-time measurements and
approve explicit budgets. Historical one-machine measurements below are retained
for comparison but are not themselves approved thresholds.

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
The current setup selects static libraries, telemetry, no gRPC server, no MAVSDK
tests, and BUILD_WITHOUT_CURL=ON. Revisit options only for a demonstrated required
plugin.

The smoke contract accepts only explicit UDP input/output endpoints and one
expected autopilot system ID. Status succeeds only while that system remains
connected and supplies valid mode, 3D-or-better GPS, battery, and at least three
position updates spanning one second with a latest-sample age at most 1.5
seconds. It fails closed for absent, wrong, ambiguous, invalid, or stale peers.

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
After the hardened smoke rebuild on 2026-09-09, the Windows Release executable
was 2,032,128 bytes and the warm working build tree was 379,886,610 bytes. These
are diagnostic measurements from one machine, not approved budgets or clean
build benchmarks.
After the dependency-input rebuild on 2026-09-10, the executable remained
2,032,128 bytes and the warm build tree measured 379,308,757 bytes. Configure,
target build, the three-case peer fixture, provenance checker, 12 focused Python
tests and 10 CTests passed. Those local measurements predate the final published
fork compatibility changes and remain historical only.

Current accepted technical evidence is the recursive hosted matrix and live SITL
runs recorded above. Run `pixi run check-mavsdk-phase-a` after any source or
dependency change, and run `pixi run build-core-mavsdk` followed by
`pixi run test-mavsdk-phase-a` for the deterministic peer contract. The reduced
mainline SITL job is the live Phase A regression gate; the larger SITL suite stays
nightly/on-demand because it exercises later safety and parity behavior.

The Phase A build task now measures configure time, target-build time, build-tree
bytes, smoke-executable bytes and selected static-archive bytes in one record.
Hosted Linux and Windows jobs upload that JSON record as a required artifact.
This makes samples reviewable but does not make a warm build a clean benchmark or
approve a budget. The live smoke job records connect/status elapsed time and
per-process-tree peak RSS on Linux and Windows, and retains its output artifact.
An unavailable RSS sample is explicit rather than silently omitted.

Two local warm Windows runs on 2026-09-10 against the pre-existing dirty vendor
checkout `34b417d4` reproduced a 445,271,541-byte tree, 2,033,664-byte executable,
six selected archives totalling 12,073,548 bytes, with configure/target-build
times of 42.074/175.365 s and 29.701/169.367 s. The provenance check failed
because that checkout's MAVLink patch lacks the reviewed pinned-generator marker.
These measurements validate the collector only; they are not accepted dependency
or release evidence and do not supersede the published `9884f109` graph.

A separate recursive Windows checkout at NOMAD `610215f` and MAVSDK
`9884f109533f564bc6250e5471e6301d3a62f4a7` passed the provenance audit,
28 focused Phase A tests, all three deterministic peer cases, 10 CTests and the
full Python suite (345 passed, 3 environment skips) on 2026-09-10. Its clean
first build completed, but its requested evidence file used an unwritable output
location, so no first-build duration is claimed. A subsequent measured warm run
recorded a 422,952,646-byte build tree, 2,032,640-byte executable, six selected
archives totalling 12,070,896 bytes, 32.695 s configure and 165.748 s target
build. This is clean-source local Windows evidence, not a hosted sample, CI-time
measurement, runtime/SITL result, approved budget or flight qualification.

Hosted run `34550522657` against commit `219133e` on 2026-09-11 retained the
following clean recursive-checkout samples and passed provenance plus all three
deterministic peer cases on both runners:

| Runner | Tree bytes | Executable bytes | Archives | Archive bytes | Configure | Target build |
|---|---:|---:|---:|---:|---:|---:|
| `ubuntu-latest` | 213,615,930 | 4,719,032 | 6 | 1,552,468 | 43.020 s | 172.291 s |
| `windows-latest` | 430,393,315 | 2,015,744 | 6 | 12,051,864 | 187.318 s | 326.074 s |

Hosted run `34550529273` against the same commit connected to the configured
Copter 4.7.1 SITL system and retained `mavsdk-phase-a-runtime`: connect completed
in 0.731 s at 9,584,640-byte peak process-tree RSS; status completed in 2.534 s
at 9,940,992 bytes and returned six fresh samples for system 1. These are hosted
dependency and live-SITL integration samples, not approved resource budgets,
aircraft-specific evidence or flight qualification.

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
Telemetry parity must also expose independently sourced AGL, metre accuracy,
per-field freshness, link-quality inputs and the official mode mapping required
by AE27-NET-002 through AE27-NET-006. The smoke's relative altitude and GPS fix
qualification are not those capabilities. Do not copy its 1.5-second age limit
into competition traffic policy or treat it as a CONOPS safety threshold.

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
Resolve Q02 before proving aircraft-phase termination. Q01 is resolved: use
the hard polygon for termination and retain the plugin internal soft inset.
Prove non-convex hard containment, 100 m AGL, independent termination, C2 loss,
five-second activation and separate rotary/fixed-wing/transition outcomes.
Parameter parity alone cannot prove a minimum 2 m/s descent through touchdown.
Preserve ArduPilot failsafes and require independent physical evidence at G7.

## Phase E — Production cutover

Switch the production connection after A-D pass. Retain golden/wire semantic
references until equivalent coverage survives replacement. Remove obsolete
codec/UDP/generation code and possibly its submodule only after caller inventory,
profile checks and stable safety traceability remapping.
Keep the old transport available for controlled comparison until both parity
and install/rollback evidence exist; remove it only afterward. A rollback package
must start disarmed/inhibited and never resume stale mission or payload actions.

Exit: default CLI/runtime and clients demonstrably use MAVSDK; full unit,
adapter and SITL matrix passes for supported firmware; dependency notices and
packaging verified. The old path cannot be a hidden runtime fallback. Competition
release G8 cannot pass with MAVSDK limited to a smoke executable.

## Phase F — Upstream work and maintenance

Prepare focused upstream changes with reproductions for confirmed ArduPilot
issues: GUIDED mode interpretation, battery units, location command semantics,
and any stop/heartbeat/fence/QuadPlane fixes. Recheck each historical finding
against pinned/upstream source before patching.

Required local fixes must have tests and an accountable maintainer even if
upstream review is pending. Each reviewable change should contain problem/
behavior, requirements, exact pins, unit/integration evidence and residual
limitations.

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
