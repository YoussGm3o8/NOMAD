# Architecture

Target design reconciled to CONOPS v1.0, 2026-09-10. Requirements and pending decisions are in
[PRD](prd.md); current implementation and discrepancies are in [migration](migration.md).
The working tree already removes Edge Core. Removal is not proof of a complete
replacement deployment.

## Ownership and runtime boundary

~~~text
Mission Planner / CLI ---- authenticated client requests ------+
                                                               v
ROS 2 / Python compute ---- validated observations ----> one C++ command owner
Competition web adapter --- validated traffic ---------> mission / safety / vehicle
                        <-- ownship telemetry ----------+       |
                                                               v
                                                    MAVLink implementation
                                                               |
                                                               v
                                                            ArduPilot
~~~

ArduPilot owns stabilization, motor control, EKF, low-level navigation and
failsafes. The competition termination mechanism belongs on the aircraft, with
independently qualified safety hardware/ArduPilot behavior even when the C++ core
or ground link is unavailable. C++ verifies configuration/readiness and exposes
outcomes; Mission Planner requests and displays, never owns parallel termination
parameter policy. All-mode containment, 100 m AGL and five-second termination
entry require separate evidence. Hard-boundary violation is the termination trigger (U-FEN-01); Q02 still
blocks final aircraft-phase termination design;
this architecture does not prescribe a new kill command or failsafe override.

The C++ core owns high-level mission behavior, command validation,
traffic response decisions, payload authorization and authoritative outcome
tracking. Perception produces observations; it does not directly steer vehicles.

The proposed persistent runtime is a thin C++ executable around the existing
library, with one connection owner, bounded work, and an explicit shutdown order.
It is justified by mission cancellation, continuous traffic/telemetry, multiple
clients and payload state. It is not implemented yet. Keep ordinary value types,
named functions and owned workers; do not add a broker, registry or general task
framework. The current one-shot CLI remains useful for exclusive local use.

| Layer | Responsibility | Excluded responsibility |
|---|---|---|
| C++ telemetry | Typed state, validity, source identity, per-field timestamps and age | ROS types, UI rendering |
| C++ vehicle/safety | Command policy, limits, verification, authority and payload interlocks | Packet layout |
| C++ mission | Survey/task progress, pause/cancel/abort, traffic responses, recovery | Perception inference |
| MAVLink implementation | Transport, target filtering, packing, ACK matching and protocol exchanges | Mission decisions |
| Competition adapter | External schema/auth, cadence, retries, bounded queues, traffic translation | Choosing maneuvers |
| ROS 2 adapter | Translate data and requests, validate transport metadata, enqueue | A second Vehicle owner in integrated mode |
| Python CV/VIO/tools | Sensor processing, tracks, candidate observations, replay and analysis | Autonomous MAVLink command path |
| Mission Planner | Operator review, maps, video, configuration, progress and diagnostics | Independent emergency/fence/payload policy |
| Routing/deployment | Link routing, process supervision, network access and packaging | Deciding whether a flight action is safe |

Target per-field freshness and command authority are missing from the current
library. The existing ROS node embeds its own Vehicle and the plugin spawns CLI
processes; those are alternative standalone modes until G2 unifies ownership.

## Boundary geometry

The hard polygon is the authoritative containment/termination boundary.
Per project decision U-FEN-01, the soft polygon is internal: offset the hard
polygon's sides inward by a configurable distance in metres, such as 5 m.
Preserve the plugin's existing SoftBoundaryFromHard / SoftBoundaryInsetMeters
behavior and UI unchanged. The internal margin does not replace or shrink the
authoritative hard fence and is not itself a termination trigger. No second
organizer polygon is required. Future integrated ownership changes must preserve
these semantics and test concave geometry, narrow regions and infeasible insets.

## Command authority and client protocol

R recommendation (D02): begin with one ground-hosted C++ runtime. In all profiles,
Mission Planner and the CLI submit typed requests to that runtime. For an onboard
core, use the same semantics through a selected authenticated remote transport;
do not spawn a local second owner when the remote core is unreachable.

Before implementation, specify a versioned request/result contract: aircraft and
session identity, request ID, deadline, verb, units, authorization scope and
outcome. Separate admitted, transmitted, acknowledged, state-verified, timed-out,
cancelled and unknown outcomes. A completed LAND command means LAND mode today;
a completed landing requires landed/disarmed evidence.

Use OS-restricted local IPC for local clients; select the exact Windows/Linux
mechanism at D02. Remote transport needs mutual endpoint authentication,
authorization, replay protection, bounded messages and revocation. VPN reachability
alone is not application authorization. No existing remote C++ command protocol
is claimed, and Python REST is not a fallback.

Maintain one active writer per aircraft. Concurrent clients may observe; requests
are serialized or rejected while incompatible work runs. Priority is explicit:
ArduPilot failsafes/manual takeover, abort, safety response, then mission requests.
Stale session requests cannot rearm, resume or repeat a payload action. Reconnect
reestablishes observation first; explicit reauthorization precedes motion.

Mission Planner native controls and RC remain possible external authorities.
Integrated operations must define handover and inhibit NOMAD until reconciled;
software cannot claim to prevent an independent pilot/autopilot action. Migrate
plugin boundary returns, emergency parameter changes, direct gimbal streams and
fence writes through the core or restrict them to documented maintenance mode.

## MAVLink and transport decision

Current production path: UdpMavlinkConnection with NOMAD framing and generated
ArduPilot dialect headers. CMake requires Python/mavgen at build time; the runtime
core has no Python or ROS dependency. Native serial/TCP are not implemented in
that connection. Existing routers can bridge them to UDP.

User-confirmed direction: adopt the pinned MAVSDK fork as an early competition
prerequisite after parity, with tests and a focused merge request, as detailed in
[MAVSDK adoption](mavsdk-adoption.md). The opt-in Phase A smoke target does not
replace the production transport. Preserve Vehicle, safety and outcome semantics
through adoption. Do not trust library ACKs, Offboard mode, background resends,
GCS heartbeats, or default identity without tests against the selected firmware.
Pin firmware/dialect/library combinations and requalify each changed combination.

MAVLink ingress must distinguish ownship from GCS and traffic, reject malformed
frames and stale state, and match acknowledgements to the pending operation.
Transport health is separate from a fresh position or a physically successful
payload action. Routing must prevent loops and duplicate command transmission.
Two radios or UDP legs do not establish independent failure protection.

## Deployment profiles

Task 1 direction is a lightweight VTOL with ground GPU CV/video and a Pi Zero
LTE backup. Task 2 is a quad below 15 kg with optional Jetson; payload and final
autonomy are TBD. Here4/RTK and Walksnail are intended components; qualification
and capture/correction interfaces remain open. There is no ZED prerequisite.

The current core hardcodes Copter modes. Add explicit aircraft-class validation
and tested ArduPlane/QuadPlane mission, transition and landing semantics before
Task 1. ArduPilot's [QuadPlane mission interface](https://ardupilot.org/plane/docs/quadplane-auto-mode.html)
distinguishes VTOL takeoff/landing and transition operations. The core should
supervise reviewed autopilot mission execution, not emulate flight transitions.
No Copter zero-velocity hold may be assumed safe during fixed-wing flight.

Compute placement and core placement are separate choices. D02 ground-core
placement below is recommended, not fixed by the user's profile definitions.

| Profile | Aircraft | Ground | Core/control constraint |
|---|---|---|---|
| onboard_companion | ArduPilot plus Jetson/SBC sensor, ROS 2, VIO/CV and video workloads | Mission Planner; competition adapter; optional ground core | Exactly one core, ground initially or explicitly selected onboard |
| groundstation_gpu | ArduPilot plus selected camera/IMU acquisition and transport; no Jetson | GPU laptop runs ROS 2/VIO/CV/video, Mission Planner, competition adapter and core | Ground core; sensor acquisition/transport hardware still required |
| groundstation_minimal | ArduPilot and required command/telemetry link | C++ core, Mission Planner/CLI, lightweight competition adapter | No companion/perception dependency; optional features report unavailable |

Ground GPU compute cannot infer vehicle motion from a stationary laptop camera.
If it processes aircraft VIO, the aircraft must deliver synchronized image and
IMU data with valid acquisition timestamps and calibrated transforms. D01/D06/D09
must establish this acquisition path and its capacity. Compressed RTSP alone is
not evidence of a usable camera/IMU VIO feed.

Select profiles explicitly. Runtime capabilities include command link,
navigation source, camera, VIO, perception, video, tracker, sampler and
competition exchange. For each expose state, reason, age and configuration
version. A mission declares prerequisites; missing perception blocks only
dependent actions, not telemetry or eligible GNSS missions.

The current velocity path requires VIO. The minimal profile does not bypass this
gate, and NOMAD_VIO_SOURCE_REQUIRED=false is not wired into that C++ policy.
Use supported non-VIO operations; a future GNSS velocity mode needs an explicit
reviewed policy and fault tests. Never synthesize healthy VIO for product use.

## ROS, VIO, perception and video

Keep ROS outside public C++ headers. Callbacks validate, translate and enqueue.
Owned workers handle blocking operations; short services return acceptance,
while long goals use actions only when a client needs cancellation/progress.
Prefer a single-threaded executor with explicit callback groups if concurrency
is introduced. The present node blocks in services and telemetry callbacks;
G2 must correct this before claiming bounded response.

Proposed observation contract: acquisition timestamp, receive timestamp, clock
domain/uncertainty, sensor identity, frame and units, calibration version,
sequence/reset counter, confidence/covariance and quality. Reject delayed,
future-dated, out-of-order, wrong-source and invalid values. Current VIO health/
confidence/source topics are not actual estimator pose or EKF fusion evidence.

For flight external navigation, translate a selected estimator into reviewed
MAVLink data through the core's MAVLink boundary. ArduPilot's
[external-navigation interface](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
documents ODOMETRY, timestamp, frame and covariance fields. Choose and qualify
those semantics for the pinned firmware, including reset, delay and origin.
No parameter recipe or estimator-source switch is approved by this plan.
Mapping-only VIO and navigation VIO are separate capabilities.

Perception returns detections and track candidates with image/time evidence,
classification confidence, association uncertainty and geolocation uncertainty.
C++ mission logic accepts/rejects candidate task actions. Survey counting
deduplicates overlapping views, records occlusion/unknown identities and permits
operator correction with provenance. Tracker radio identity and visual identity
must not be silently equated.

Video remains outside the control loop: selected camera/ROS source, encoder,
MediaMTX/RTSP or another justified transport, then Mission Planner playback.
The retained Python bridge moves images; it is not the removed vehicle service.
Preserve acquisition timestamps separately from presentation time, show frozen/
stale video, bound buffers, and prioritize control/traffic over video bandwidth.
Its current HTTP control endpoint lacks authentication; constrain it at G1 and
qualify it at G3/G8. Overlay switches are not evidence of working detection.

## Competition telemetry and traffic

Use one ground-side competition adapter, initially C++ as an application adapter
outside the reusable core. The wire protocol, library, authentication and schema
remain D07. A mock server can use Python because it is a test fixture.

Outbound telemetry is required at 1 Hz whenever armed in Task 1. The confirmed
fields are bidder UAV ID, Unix time, decimal-degree position, AGL metres,
horizontal/vertical accuracy metres, battery percent, six-state official mode
and normalized RC/telemetry link quality (AE27-NET-001 through AE27-NET-006).
Core telemetry owns source validity, age and aircraft state; the adapter maps to
the versioned wire contract. Neither MSL nor home-relative altitude is AGL without
a reviewed terrain/reference conversion. GPS fix/satellite count is not metre
accuracy; a battery-valid flag must not validate an unknown percentage.

Track scheduled, sent and server-received time separately. Bounded retries and
expiry are proposed engineering policy pending Q04; never send historical
positions as current or replay a burst to fill a 1 Hz gap. The penalty thresholds
in the PRD inventory are scoring rules, not safe age limits. Do not hardcode
unverified endpoints, field names, timestamp precision or authentication flow.
Disarmed initialization is a project strategy to avoid startup-armed penalties,
not a CONOPS-specified disarmed cadence. Event uploads, their schema and delivery
semantics are unconfirmed; implement no event wire protocol before Q04. Internal
mission evidence and physical action IDs remain necessary regardless of server
support. Task 2 CSV is a separate ground deliverable, not assumed server telemetry.

Inbound traffic is separate from ownship. Validate schema, vehicle identity,
timestamp, coordinates, datum, velocity, validity and sequence where supplied.
Record receive age and uncertainty; handle duplicates, gaps, reordering, stale
tracks, server errors and reconnects. Simulated-UAV cooperation currently means
receiving traffic and avoiding its
exclusion zones; no additional cooperation-event contract is confirmed.

C++ deconfliction starts as deterministic advisory logic (initial D05 scope confirmed): compare
ownship plan/state with server-supplied cylindrical exclusion zones, account
for age/uncertainty, and report conflict interval, source and reason. A stale feed
means traffic unknown, not clear. Server radial/vertical keepaway values must
be respected; do not invent a
constant separation radius. Vertical extent/datum semantics, freshness,
prediction horizon, right-of-way and loss-of-feed response are Q04/D08.
A proposed prediction horizon is an engineering choice requiring evidence.
Advisories must produce demonstrably timely operator avoidance; displaying a
warning alone does not satisfy AE27-NET-008. Manual operation is permitted by
AE27-OPS-028, but that permission does not waive the exclusion zones.
Automatic reroute/hold/RTL/land needs feasibility, fence, terrain,
energy and command-authority checks plus separate evidence. A zero velocity
command or RTL is not universally collision-safe. Simulated traffic is not
real-world detect-and-avoid certification.

## Tracker and sampling payloads

Split tracker device firmware/hardware, tracker data adapter, C++ task state and
operator presentation. Select the actual radio/GNSS/protocol at D04; receive
positions independently of the vehicle link, track sequence/age/battery and
identity and clock provenance, map gaps, and preserve task association across
restarts. Task 2 requires a chronological ISO8601 timestamp/latitude/longitude
CSV before window end. Record the five-minute tracking interval starting at
100 m withdrawal and assess interpolation against independent ground truth.
The core enforces the moving-target 100 m horizontal offset throughout all
remaining flight actions, including sample approach and return. Missing/stale
tracker positions cannot mean the offset is satisfied. Tracker device/radio
identity is distinct from ownship and simulated-aircraft identity.

The tracker must be custom, under 250 g and at most 8 cm on every axis, standalone
and untethered. One attachment only: hook-and-loop (hook on tracker) or allowed
box placement. Payload feedback must distinguish release from actual attachment.
Sample interfaces must support observed egg integrity, dung core dimensions/
colour/shape and droppings count/integrity, not merely a relay activation.
A task evidence exporter is a ground tool; C++ owns task/attempt selection and
authorization, Python may format/review data without becoming a vehicle owner.

C++ payload operations bind authorization to task, target, output and expiry.
Proposed states: safe, authorized, executing, verified, failed/unknown. Cancel,
restart or takeover invalidates permission. Battery-swap recovery is conditional
on organizer permission Q03; no plan assumes it is allowed. Require observed
attachment/sample
feedback or explicit operator confirmation; an ACK only establishes command
acceptance. Never automatically retry an uncertain irreversible action.

Generic servo/relay/user-command paths currently bypass the dedicated
release_payload interlock. G2/G6 must reserve hazardous channels and route all
their access through the same policy. Hardware pulse timeout and safe power-loss
behavior are required evidence; a ground-side timer cannot guarantee relay-off
over a broken link. Do not grow Python payload decision logic.

## Observability, security and scope ceilings

Record bounded structured events: command/request/session, source, safety reason,
state transition, telemetry age, traffic age, capability loss, payload outcome,
dropped frames/queues, process restart and server receipt. Keep per-field units
and clock provenance. Rotate logs, surface disk-full, protect credentials and
animal location data, and retain sanitized replay artifacts for release gates.

Security implementation gaps are explicit in [safety](safety.md) and
[operations](operations.md). No authentication or audit guarantee is inherited
merely by linking the library.

debt: one aircraft command owner per task; revisit when CONOPS requires control
of multiple real aircraft; then define per-aircraft ownership before adding
fleet coordination.

debt: advisory traffic baseline; revisit when CONOPS or approved autonomy
requires maneuvers; then qualify bounded response logic with independent traffic
and fault evidence.

debt: one-command CLI for isolated operation; concurrency already triggers the
ceiling; then add the G2 persistent runtime without replacing the core API.
