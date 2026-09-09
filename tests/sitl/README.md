# SITL scenario suite

These tests drive isolated ArduPilot SITL and observe authoritative vehicle state.
Normal pytest skips live scenarios without an explicitly configured simulation.
Nightly/on-demand SITL CI is configured, but a workflow file is not passed-run
evidence; current startup still needs the G1 Edge Core reference repair.

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

The old sitl-gimbal task references a missing gimbal_mount_control.py; it is a
G1 repair item, not a runnable test. No successful gimbal evidence is claimed.

Use the commands and safety discipline in
[development](../../docs/development.md) and
[operations](../../docs/operations.md). Run scenarios serially against a known
disarmed isolated simulator; never reuse a hardware endpoint for fault injection.
A configured passive observer link can feed Mission Planner without issuing
commands.

Existing scenarios are Copter-oriented. Add separate QuadPlane takeoff,
transition, cruise, return and VTOL landing evidence for Task 1. Required gate
artifacts and historical/current distinctions live in
[migration](../../docs/migration.md); do not duplicate pass counts here.
