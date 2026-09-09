# Product requirements

Baseline: 2026-09-08. NOMAD is a C++20 vehicle-control system with independent
clients and optional companion compute, prepared for the AEAC SUAS 2027 Wildlife
Monitoring assessment preview.

## Requirement authority

| Label | Meaning |
|---|---|
| U | Explicit product direction from the project owner in this planning request |
| P | Competition preview requirement supplied by the project owner; provisional until CONOPS reconciliation |
| E | Independently checked organizer publication |
| R | Architect recommendation; not a competition rule or accepted user decision |
| TBD | Unresolved; cannot be used to authorize behavior or close a gate |

The [organizer page](https://www.aerialevolution.ca/2027-student-competition/)
was checked on 2026-09-08. It confirms the wildlife survey/tagging scenario and
announces mid-September CONOPS publication. It links the
[assessment preview](https://drive.google.com/file/d/1jsKWHc171YceXCohEF51jnElIylXg505/view).
The linked PDF body was not retrievable through the browsing tool; P rows below
use the user's supplied summary, not a claimed independent reading of the PDF.

No full CONOPS has been baselined here. At G0, record its version/date and section
for each P row, reconcile differences, and update affected gates before treating
new details as requirements. Do not import old CONOPS section numbers or scoring
rules from source comments. Confirmed product direction and provisional preview
requirements are intentionally distinct.

## Competition requirements

| ID | Basis | Requirement | Acceptance evidence |
|---|---|---|---|
| P-T1-01 | P | Long-distance herd survey; visual detection, counting, identification | G5: ground-truth survey dataset and field rehearsal, coverage/count/identity errors reported |
| P-T1-02 | P | Cooperate with simulated UAVs; exchange telemetry and deconflict traffic | G4/G5: independent traffic simulator, cooperation events and conflict scenarios |
| P-T1-03 | P | End Task 1 with safe landing | G5/G7: authoritative landed/disarmed state and operator evidence |
| P-T2-01 | P | Design a tracker and tag the animal | G6/G7: tracker qualification, association and observed tag placement |
| P-T2-02 | P | Track animal movement and map its path | G6: independent trajectory, position error and gaps visible |
| P-T2-03 | P | Collect simulated biological samples | G6/G7: measured collection result, identity, containment and abort tests |
| P-AIR-01 | P | One aircraft per task | G7/G8: task roster and configuration; same airframe across tasks is TBD |
| P-AIR-02 | P | Aircraft mass below 15 kg including payload | G7: calibrated all-up measurement of each flight configuration, including battery |
| P-OPS-01 | P | Battery swaps allowed only for Task 2 | G5: Task 1 single-battery endurance; G6: Task 2 safe swap/resume |
| P-NET-01 | P | Send telemetry to the competition web server at 1 Hz plus event-driven data | G4: mock and official server receipt timing, schema validation, event reconciliation |
| P-NET-02 | P | Receive and process a 1 Hz traffic stream | G4: timed ingestion, expiry, malformed/reordered/duplicate/lost-update tests |

Task distances, duration, animal classes and appearance, identification meaning,
tag attachment method, sample medium/quantity, scoring, minimum separation,
altitude datum, allowed autonomy, server protocol/authentication and delivery
tolerances remain TBD. Do not invent those values. Strictly below 15 kg is the
supplied constraint; manufacturing/measurement margin is a design decision.

## Confirmed product direction

| ID | Basis | Requirement | Gate |
|---|---|---|---|
| U-CORE-01 | U | C++ owns vehicle behavior, safety validation, missions, telemetry models and MAVLink interaction | G2 |
| U-AP-01 | U | Preserve ArduPilot control, EKF and failsafes; NOMAD does not replace them | G2/G7 |
| U-PY-01 | U | Remove Edge Core and Python-owned vehicle decisions; retain Python for CV/ML/tools/tests | G1/G2 |
| U-ADAPT-01 | U | Mission Planner and ROS 2 are clients/adapters; no parallel vehicle policy | G2 |
| U-PROF-01 | U | onboard_companion runs optional ROS 2, VIO, camera/video and perception on a Jetson/SBC aboard | G3/G7 |
| U-PROF-02 | U | groundstation_gpu runs those optional workloads on a GPU laptop with no Jetson aboard | G3/G7 |
| U-PROF-03 | U | groundstation_minimal works with direct MAVLink and C++ without companion/perception; missing features are explicit | G3 |
| U-TEST-01 | U | Important behavior is testable without hardware; safety needs invalid, boundary and failure cases | All |
| U-SCOPE-01 | U | This iteration reconciles plans and docs; no speculative implementation or destructive migration | G0 |

Board-independent MAVLink is the design boundary. Current verification is
Copter-oriented; this is not a claim that every ArduPilot vehicle type, board,
payload channel mapping, or transport has been qualified.

## User direction recorded during review

- Task 1: proposed lightweight VTOL, no onboard Jetson, groundstation_gpu for
  CV/video; Pi Zero for backup LTE and possibly video streaming.
- Task 2: heavier quadcopter in the 15 kg class, still strictly below 15 kg
  all-up; Jetson onboard is optional pending the actual autonomous task.
- Walksnail FPV camera planned; additional camera TBD. No ZED dependency.
- Use the flight controller's IMU. Cube Orange or a custom ArduPilot controller
  is under consideration; UART/PWM counts and board integration need validation.
- Here4 GNSS with RTK base is intended; correction delivery and degraded-fix
  behavior require qualification.
- Custom tracker, possibly ESP32, remains undecided. Tagging and sample
  mechanisms/feedback are not selected.
- Begin with traffic advisories and explicit action authorization. Revisit
  automatic traffic/payload behavior after CONOPS; it is not yet required.
- MAVSDK must be used at competition: early adoption, unit/integration testing
  and a focused merge request are required implementation work (D03 resolved).

These are user product directions or explicitly tentative hardware choices,
not additional organizer rules. ArduPlane/QuadPlane support is necessary for
the proposed Task 1; Copter evidence cannot establish it.

## Mission workflows

Task 1: load reviewed survey area and limits; check energy and required
capabilities; survey; accept timestamped detections with evidence references;
associate tracks and avoid double counting; obtain operator review where needed;
exchange cooperation events and traffic; return with reserve; verify landing;
export the reviewed result and its evidence.

Task 2: register tracker and target association; authorize a tagging attempt;
observe attachment or declare outcome unknown; receive tracker positions and
map gaps; authorize simulated sampling; verify collected material and its
identity; return and disarm. For a battery swap, preserve mission/tracker/sample
records, invalidate action permissions, recheck aircraft state, and require
explicit resume. Never replay an uncertain physical action after restart.

These detailed sequences are R proposals within the user's advisory/authorization
scope. They do not establish automatic targeting,
animal approach distances, payload mechanics, or a permission to interact with
real wildlife. Payload/assessment interpretation is D04.

## Performance and capability contract

P-NET-01/02 establish rates, not latency, jitter, availability or safety margins.
Before implementation acceptance, choose measurable budgets for telemetry age,
traffic expiry and lookahead, command cancellation, inference latency, video age,
count/identity accuracy, map error, link capacity and endurance reserve (D07/D08).

Every optional feature reports one of: unconfigured, unavailable, starting,
ready, degraded, or failed, with reason and observation age. This is an R target
contract, not an implemented capability service. Readiness requires runtime
evidence, not GPU discovery or an environment boolean. Minimal operation supports
eligible non-perception missions; competition task readiness is evaluated
separately and can legitimately be unavailable.

## Decisions needing team input

Unanswered questions stay pending. Recommendations below may guide prototypes,
but cannot authorize autonomous actions, runtime changes, or release acceptance.

| ID | Decision | Recommendation and consequence | Needed before |
|---|---|---|---|
| D01 | Exact VTOL/quad firmware, hardware and Task 2 core/Jetson placement | Task 1 ground GPU and two aircraft types now directed; exact integrations remain TBD | G3/G7 |
| D02 | Core placement and client transport, groundstation OS | Persistent ground core for first integrated release; onboard authority only when required and remotely authenticated | G2/G3 |
| D03 | MAVSDK cutover before competition | Resolved: mandatory and early priority, with tests and a focused merge request; G-M gates production cutover | G-M/G8 |
| D04 | Tracker/tagger/sample hardware and assessment interaction | Select mechanics and feedback before defining autonomous payload behavior; current generic outputs are insufficient | G6 |
| D05 | Final traffic, approach and payload autonomy level | Initial scope resolved: advisories and explicit authorization; revisit greater autonomy after CONOPS | G4/G6 |
| D06 | Is VIO for mapping/perception or required flight navigation? | GNSS/ArduPilot navigation baseline; external-navigation fusion only after end-to-end timing evidence | G3/G7 |
| D07 | Official server contract, event types, simulated-UAV cooperation and separation rules | Versioned adapter and mock now; no invented wire schema or separation threshold | G4 |
| D08 | Accuracy, latency, stale-data, reserve and operating-environment budgets | Agree numeric acceptance thresholds before collecting gate evidence | G3–G7 |
| D09 | Primary radio, communications topology and manual authority | Pi Zero LTE backup is intended for Task 1; primary link and VPN/ELRS choices remain TBD; measure common-mode failures | G3/G7 |
| D10 | Named engineering, payload, perception, safety and test owners; capacity and dates | Assign accountable people to gates before promising a schedule | G1 |
| D11 | Evidence storage, retention and team/server credentials | Access-controlled artifacts; sanitized manifests in repository; no private datasets or secrets committed | G4/G8 |

User answers above resolve D03 and the initial D05 scope, and partially resolve
D01/D04. Other entries remain open. All competition-rule interpretations must
be reconciled with CONOPS.

## Scope boundary

No generic plugin registry, event bus, mission scripting language, distributed
workflow platform, or replacement autopilot. One controlled aircraft per task
does not imply owning a multi-aircraft fleet; simulated traffic uses independent
track identities. Video display does not imply VIO navigation. Removing a Python
vehicle service does not remove supported onboard or ground GPU compute.

Implementation status and executable evidence live in [migration](migration.md);
safety requirements live in [safety](safety.md). Do not duplicate pass counts here.
