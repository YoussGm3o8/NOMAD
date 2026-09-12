# SITL scenario suite

These tests drive isolated ArduPilot SITL and observe authoritative vehicle state.
Normal pytest skips live scenarios without an explicitly configured simulation.
Nightly/on-demand SITL CI is configured, but a workflow file is not passed-run
evidence; the repaired startup path still needs a current live G1 run.

## Local test responsibilities

- velocity_loop_closure.py: C++ CLI velocity command, observed motion/stop and
  mode-gate checks; Python is a test driver/observer.
- test_velocity_loop_closure.py: environment-gated pytest wrapper.
- scripts/dev/core_sitl_containment.py: in-fence movement, rejected out-of-fence
  target, authoritative position and cleanup (sitl-fence).
- scripts/dev/core_sitl_zero_delivery.py: independent wire zero plus observed
  vehicle stop; total physical link loss is a separate test.
- Other core_sitl_* runners cover status, command flow, mission, fence
  upload/readback, payload, link recovery and heartbeat relay behavior.

The obsolete sitl-gimbal task was removed with runtime wiring repair. No successful gimbal evidence is claimed.

Use the commands and safety discipline in
[development](../../docs/development.md) and
[operations](../../docs/operations.md). Run scenarios serially against a known
disarmed isolated simulator; never reuse a hardware endpoint for fault injection.
A configured passive observer link can feed Mission Planner without issuing
commands.

Scenarios share one vehicle, and two of them leave state that changes what a
later one can do. `core_sitl_geofence.py` uploads a polygon fence that stays
active even after it restores `FENCE_ENABLE`, and an active polygon makes
ArduPilot refuse guided targets outside it (observed: the same reposition
accepted with `FENCE_ENABLE=0` and rejected with the polygon loaded), so run
guided-flight scenarios before the fence scenario or reset the vehicle between
groups. `velocity_loop_closure.py` returns the vehicle with RTL without waiting
for the landing disarm, so a scenario that requires a disarmed vehicle can start
while it is still armed; wait for disarm or reset between those two.

One unexplained failure is recorded here rather than explained away: on
2026-09-12 `core-sitl-link-recovery` failed on a fresh stack immediately after
`core-sitl-link-loss`, timing out on `wait_for_status({"armed": "true"})` while
its own `arm` command had exited zero, so the CLI believed it had verified the
armed state and the following 15 s of status reads did not agree. Re-running the
same pair, and the same four-scenario order, passed twice afterwards, so the
cause is not identified: treat a repeat as a real signal and capture the arm
step's full output before retrying.

Existing scenarios are Copter-oriented. Add separate QuadPlane takeoff,
transition, cruise, return and VTOL landing evidence for Task 1. Required gate
artifacts and historical/current distinctions live in
[migration](../../docs/migration.md); do not duplicate pass counts here.
