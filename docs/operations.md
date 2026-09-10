# Operations

Target procedures reconciled to CONOPS v1.0, 2026-09-10. Profile selection does not
authorize hardware use or prove readiness. [Migration](migration.md) owns gates;
[architecture](architecture.md) owns component boundaries.

## Task and deployment matrix

| Task/profile | Aircraft-side functions | Ground-side functions | Qualification status |
|---|---|---|---|
| Task 1 / groundstation_gpu | Proposed lightweight VTOL; Walksnail FPV camera; Pi Zero for backup LTE and possibly video; Here4 GNSS | GPU CV, video capture/display, C++ core, Mission Planner and server adapter | User direction; video capture, RF bandwidth, QuadPlane support and endurance unqualified |
| Task 2 / onboard_companion candidate | Under-15-kg quad; optional Jetson if useful autonomy emerges; tracker/tagging/sampling payload | Mission Planner, operator approvals and tracker CSV export; server applicability Q04 | Jetson placement and payload mechanism TBD |
| groundstation_minimal | ArduPilot, navigation and selected command/telemetry link | C++ core and clients; lightweight server exchange when implemented | Product requirement; no ROS/perception dependency |
| Development | Isolated Copter and future QuadPlane SITL | Core, fake/mock services and passive observers | Local build/config checks pass; live startup and profile qualification remain open |

Use a strict under-15-kg project ceiling including maximum task payload; the
CONOPS maximum-15-kg wording conflicts with its under-15-kg FRR wording (Q03).
Weigh every configuration and budget measurement margin. Task 1 has no battery
swap. Task 2 swap permission is unstated: plan single-battery completion until
organizer clarification, retaining swap recovery only as a conditional design.

A Cube Orange or custom ArduPilot controller is under consideration. UART/PWM
availability, power budget, timing, firmware support and channel mapping require
board-specific qualification. Here4 with an RTK base is the stated navigation
choice; verify correction transport, fix state and age, and behavior without
RTK corrections. Walksnail receiver capture/export interface and any second
camera remain to be selected/tested. The plan has no ZED dependency.

## Competition runbook requirements

The [PRD inventory](conops-requirements.md) owns exact rules and scores. This
runbook translates them into preparation, not a flight authorization.

1. Complete eligibility, registrations, insurance and per-aircraft FRR documents;
   obtain judge acceptance and schedule the props-off termination demonstration.
   Keep private certificates and receipts outside source control. Verify the hard polygon and
   resolve Q02 aircraft-phase termination before loading a competition fence.
2. Prepare the proposal by 2027-01-15 1700 ET, FRR by 2027-04-28, bilingual
   presentation by 2027-05-13 2359 ET, and mission report by 2027-05-26 1700 ET.
   Team/member registration dates are 2026-11-27 and 2027-03-12. Follow all format,
   page, heading and feedback-attestation conditions in AE27-ADM requirements.
3. Receive official task geometry/course by Thursday night; validate non-convex
   boundaries, AGL limits, route feasibility, payload search areas and Task 2
   100 m moving-target offset. Rehearse with variable windows; approximately
   30 minutes is not a guaranteed duration or an endurance budget.
4. Arrive with all equipment at least 30 minutes before window (scored rule).
   At most five flight crew at line; no communication with other team members
   during window. No field access without RSO permission. Use ground propeller
   inhibit, effective checklists and explicit per-aircraft pilot assignments.
5. Keep transmitters off until window or explicit FRR authorization. Obtain RSO
   permission before each takeoff. Display live position and competition area
   on a dedicated GCS display for each aircraft. Report GPS/link/RC/fence anomalies.
6. Task 1: establish server session before arming as the selected startup strategy;
   send required 1 Hz telemetry whenever armed; fly complete lap course then
   survey. Monitor cylinder separation and operator response deadlines. Submit
   labelled `<your_team_name>_task1_survey.txt` in Phase 2 Deliverables; preserve
   Drive receipt, with hard cutoff 15 minutes after window. Earlier submission
   earns the stated multiplier. Land safely and clear all UAS parts by window end.
7. Task 2: verify one permitted tracker placement, withdraw at least 100 m, then
   maintain that horizontal offset from the moving target through sample pickup
   and flight-line return. Record five-minute tracker path and submit chronological
   CSV before window ends. Return intact samples onto the blue 32-inch pad before
   cutoff. Manual unloading is allowed; record intervention honestly for scoring.
   Land safely, leaving only the one attached tracker in the field.
8. Stop transmissions at window end. A restart forfeits prior-attempt points;
   reconcile selected attempt and do not merge prior evidence silently. Do not
   publish task setup/perceived performance until every window finishes unless
   Chief Judge permits it. No on-site rehearsal flights without permission.

Use the existing plugin soft-from-hard feature unchanged: the soft boundary
is an internal configurable inward margin, such as 5 m, from the hard polygon
(U-FEN-01; Q01 resolved by the project owner). A soft-margin crossing is not itself
a termination trigger; hard-boundary violation requires termination. No separate
official soft polygon is required. Lost termination control must
cause independent aircraft self-termination; ordinary link failover cannot waive
that rule. Safety lead must approve the exact definition of surviving C2 and prove
it for the selected radio/LTE topology. RTK, video and competition-server outages
are distinct faults from loss of the termination path.

## Profile and configuration lifecycle

The profile files and scripts/profile.py exist. The three product names are
onboard_companion, groundstation_gpu and groundstation_minimal. dev, drone and
the older groundstation template are migration artifacts, not extra product
architectures. Run profile-list to inspect template names; profile-load writes
ignored runtime configuration and also changes Mission Planner configuration.
Do not load a profile just to read it.

The templates use explicit C++ UDP endpoints and contain no deployment key.
Invalid endpoints and mismatched profile identity fail before activation; profile
switches clear stale owned Mission Planner settings. Optional ROS, video and
container services remain disabled until their image, camera and estimator are
qualified. NOMAD_VIO_SOURCE_REQUIRED=false does not bypass the C++
VIO-conditioned velocity gate. A capability flag does not establish a healthy
sensor. G3 must validate effective configuration end to end.

Target activation: choose profile, aircraft/firmware identity, single core host,
command transport, navigation capability, video source, payload mapping and
server config independently; validate them; then activate while disarmed.
Reject invalid/missing required settings. Record sanitized configuration version
and capabilities; keep real settings/credentials in ignored local storage.

## Connection behavior

Current C++ transport accepts udp, udpin and udpout endpoint schemes, such as
udpin:127.0.0.1:<port>. Native serial/TCP are future MAVSDK deployment capabilities,
not working current CLI transports. Routers bridge selected physical links to
UDP. Core placement does not follow automatically from compute placement.

NOMAD_RELAY_ADDRESS optionally selects the pre-latch GCS announcement destination.
The existing UDP code emits 1 Hz GCS heartbeats while waiting/polling; after peer
latching it uses the peer. Malformed overrides fail connection. This heartbeat
is unrelated to the competition's required 1 Hz telemetry upload.

For remote core placement, a secure network and an authenticated client protocol
are both required; the latter is unimplemented. Until G2, use one explicitly
selected standalone writer with exclusive test ownership. Running the ROS node
and plugin-spawned CLI simultaneously is not a qualified integration.

LTE via Pi Zero is the user's proposed Task 1 backup link, possibly carrying
video. The primary command/telemetry link is D09. A link budget must separate
command latency, telemetry, RTK corrections and optional video; LTE/video load
must not starve safety messages. Independent RC and a second telemetry path
must be assessed for shared power, antenna, spectrum and router failures.

## Startup, degradation and recovery target

1. Establish physical safety, correct aircraft and firmware, configuration
   version, selected core owner and manual control authority.
2. Start routing and the core; verify ownship identity, fresh individual state,
   reviewed limits, fence readback and payload-safe state.
3. Start only selected adapters. Verify camera acquisition, clocks/calibration,
   CV/VIO/video health, tracker feedback and server connectivity as required.
4. Display capability reasons and ages; permit only missions whose prerequisites
   are met. Obtain explicit payload permission separately.
5. Record mission state and evidence; monitor data age, link margins and energy.

| Failure | Required behavior |
|---|---|
| Video/CV lost | Mark unavailable/stale; stop dependent task decisions; keep eligible flight/telemetry functions |
| VIO lost | Refuse/stop VIO-dependent control; use only separately qualified navigation/recovery |
| RTK corrections lost | Show changed fix quality/age; apply reviewed navigation accuracy policy |
| Traffic/server lost | Mark traffic unknown and delivery impaired; apply approved task loss-of-feed procedure |
| One link lost | Continue only with termination authority intact and measured surviving capacity; otherwise use approved self-termination behavior |
| Termination/C2 path lost | Aircraft must self-terminate under the approved all-mode mechanism; a ground stop attempt or ordinary RTL is insufficient evidence (AE27-OPS-015/019) |
| Core/client restart | Reconcile authoritative aircraft/task state; expire permissions; no automatic motion resume |
| Payload outcome uncertain | Mark unknown, inhibit retry, inspect/reconcile physical state |
| Task 2 battery swap, only if permitted (Q03) | Land/disarm, make payload safe, preserve records, reset permissions, preflight and explicit resume |

These are target rules. Numeric deadlines and aircraft-specific abort actions
need D07/D08 and safety approval. A resumed heartbeat does not automatically
resume a mission, and fixed-wing motion cannot be made safe by a Copter zero
velocity command.

## Simulation and test operations

Read [development](development.md) before running tasks. Current known-good local
checks include core/Python tests, retained-package checks and daemon-free Compose
resolution. `dev` and `dev-build` build the C++ core; bootstrap and Jetson setup
use the product profile manager and retained systemd inventory. Remote setup
requires a pre-trusted SSH host key and does not open an HTTP API port. Live image,
SITL and ROS runs still require G1 qualification
before `dev-up` or `sitl` can be treated as verified quickstarts.

SITL runners already exist for status, command-flow, mission, watchdog, fence,
payload, link loss/recovery, GCS-heartbeat and zero-delivery. Run them serially
against an isolated identified simulation to qualify the repaired startup. Verify
disarmed/known state between scenarios. Fault injection must never target a
real aircraft by accidental endpoint reuse. A passive Mission Planner observer
may use the configured simulator TCP observer link; it must not issue commands.

Use a separate QuadPlane SITL vehicle for Task 1 transitions and return/landing.
Existing Copter runs do not qualify it. Add mock competition telemetry/traffic and
recorded image/tracker feeds before demanding GPU simulation. Gazebo/Isaac are
optional when sensor/physics evidence requires them, not core build dependencies.

## Observability and evidence

Display ownship and each field's age, aircraft type/mode, command owner/outcome,
mission progress/cancellation, navigation/RTK state, traffic feed age/advisories,
server delivery backlog, video age, tracker identity and payload safe/unknown
state. Log transitions with correlation IDs, configuration versions and clocks.

Measure update cadence/jitter, stale events, lost/reordered traffic, queue depth,
command latency, watchdog timing, CPU/GPU/memory/thermal headroom, dropped frames
and disk use. Logs rotate and disk-full has an explicit alert. Store imagery,
flight logs and tracker paths with access controls and retention chosen at D11;
repository evidence manifests are sanitized and reference private artifacts.

## Security and packaging

The current CLI accepts any nonempty NOMAD_API_KEY: a local opt-in, not identity
verification. OS account/file/IPC permissions are the immediate trust boundary.
Production requires the G2 authenticated/authorized protocol, bounded parsing,
session/replay controls and complete audit. Enable and test DDS security for any
exposed ROS command surface; no such protection is implied by ROS domain naming.

Separate competition credentials from local command credentials. Validate TLS
and server identity for the selected official protocol. Restrict media HTTP,
RTSP, SSH and MAVLink endpoints to intended peers. The retained media HTTP server
lacks authentication and therefore rejects non-loopback binds. Never ship
development credentials as production configuration or commit real hosts/keys.

Release packages contain the tested core/plugin/adapters, dependency notices,
configuration templates and procedures. Qualify OS/architecture/GPU drivers and
firmware pairs; test clean install and rollback. Plugin build/install scripts
may overwrite an installed plugin, and profile-load changes local runtime state.
Perform those only as separately authorized operations. No runtime infrastructure
was changed by this planning review.
