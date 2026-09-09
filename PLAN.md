# NOMAD delivery plan

Planning baseline: 2026-09-08. This replaces the duplicated migration narratives
previously in PLAN and TODO. It plans software for the AEAC SUAS 2027 Wildlife
Monitoring preview; it does not approve flight, hardware changes, or deployment.

## Read this plan in order

1. [Product requirements and decisions](docs/prd.md): requirement provenance,
   competition scope, assumptions, and the decisions requiring team input.
2. [Architecture](docs/architecture.md): ownership and data/control boundaries.
3. [Migration and release gates](docs/migration.md): source-backed inventory,
   contradictions, dependencies, and objective acceptance evidence.
4. [Safety case](docs/safety.md) and [operations](docs/operations.md): failure
   behavior and how each profile is qualified.
5. [TODO](TODO.md): the single actionable ledger.

Each subject has one canonical owner above. Component READMEs explain local
interfaces. [MAVSDK adoption](docs/mavsdk-adoption.md) is a subordinate transport
decision record, not another product roadmap. Historical CHANGELOG entries are
history, not current release evidence.

## Recommended system

One active C++ command owner per aircraft owns vehicle decisions, mission state,
traffic response, payload authorization, telemetry validity, and command outcomes.
ArduPilot owns stabilization, EKF, navigation execution, and independent failsafes.
Mission Planner presents operator workflows; ROS 2 and Python provide observations
and optional compute. The competition web adapter exchanges data without owning
vehicle decisions.

A small persistent C++ runtime is now justified by concurrent clients, the 1 Hz
competition exchange, traffic processing, cancellation, and payload state. Keep
the existing library and CLI; introduce no generic framework or Python vehicle
service. Local one-shot CLI use remains useful for isolated tests.

Support all three product profiles: onboard_companion, groundstation_gpu, and
groundstation_minimal. A profile describes compute placement, not control
authority, navigation validity, or competition readiness. Minimal operation must
work without ROS/perception, but does not by itself satisfy wildlife assessment.

## Delivery sequence

| Phase | Outcome | Exit gate |
|---|---|---|
| Planning | Reconciled requirements, source inventory, decisions explicitly pending | G0 |
| Baseline repair | Build/CI/tasks match the deleted Edge Core tree; retain optional compute and video | G1 |
| MAVSDK adoption | Tested ArduPilot transport parity, production cutover and focused merge request | G-M |
| Core authority | One active writer, fresh state, cancellable missions, bounded adapters, truthful outcomes | G2 |
| Profile qualification | Each profile runs its declared capabilities and exposes missing ones | G3 |
| Competition integration | Outbound 1 Hz telemetry/events, inbound 1 Hz traffic, deterministic advisories | G4 |
| Task 1 | Survey evidence, count/identity review, simulated-UAV coordination, return and landing | G5 |
| Task 2 | Tracker association/path, authorized tagging and simulated sampling, battery-swap recovery | G6 |
| Hardware qualification | Mass/endurance/link/sensor/payload evidence for selected aircraft and profiles | G7 |
| Release | Full rehearsals, security/observability, operator acceptance and reproducible package | G8 |

G4 can use recorded/synthetic observations before G5/G6. Payload and sensor bench
development may proceed after the relevant software fault gates; it cannot close
G7 early. Autonomous conflict maneuvers and external-navigation fusion have their
own safety evidence inside G4/G7, even if advisory traffic or CV already works.

User decision (2026-09-08): MAVSDK must be used during the competition and is an
early prerequisite, including unit tests, integration evidence and a focused
merge request. Complete G-M after baseline repair and before dependent integrated
command work. Phase A is partial and the current codec still runs; switch only
after parity. Isolated server/CV prototypes may proceed independently. Never run
two production command owners as an adoption experiment.

Task 1 targets a lightweight VTOL with groundstation_gpu, ground CV/video and
Pi Zero LTE backup. Task 2 targets a heavier quad below 15 kg with optional onboard
Jetson. The current Copter-only mode semantics need explicit QuadPlane support.
Traffic advisories and explicit payload authorization are the initial scope;
greater autonomy remains a CONOPS decision. Hardware detail is recorded in PRD.

## Planning milestones

The [organizer page](https://www.aerialevolution.ca/2027-student-competition/)
lists proposal submission on January 15, flight demonstration on May 14–16,
and mission report on May 26, 2027; it says CONOPS publication is planned for
mid-September 2026. Revalidate these external dates when baselining the CONOPS.

Recommended internal sequence: close requirement and hardware choices after
CONOPS review; substantiate the proposal with measured feasibility; complete
integration and authorized rehearsals before the May demonstration; preserve
evidence for the report. These are dependencies, not staffing estimates or
promised dates. Team owners and capacity remain D10.

## Working discipline

The planning/migration baseline is recorded in commit `d31b0aa`. Focused
implementation follows the single active item in TODO; build/task repair is
underway with runtime limitations recorded in migration. Preserve unrelated work
and follow AGENTS for publication and deployment authorization.

Gate closure requires a recorded artifact/configuration and independent observed
outcome. A source file, a configured CI job, a mock response, or an old pass count
is not a current integration result. See the evidence contract in migration.
