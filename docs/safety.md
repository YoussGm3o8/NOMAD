# Safety case

Baseline: 2026-09-08. This is a safety argument and verification backlog, not a
flight authorization. Existing requirement IDs remain stable. Current source and
test results are in [migration](migration.md); proposed requirements below are
not claimed as implemented.

## Safety argument and limits

NOMAD validates high-level requests and observes their outcomes. ArduPilot owns
stabilization, EKF, low-level navigation and its independent failsafes. Loss of
NOMAD, ROS, perception, video, ground compute or competition connectivity must
not suppress those failsafes.

There is no universally safe command for every aircraft state. A zero velocity
attempt can stop a Copter guided stream only when delivered and accepted; it is
not a fixed-wing abort maneuver. RTL or land must be appropriate to aircraft
type, navigation health, terrain, traffic, fence and remaining energy. Task 1's
proposed VTOL requires its own ArduPlane/QuadPlane safety evidence.

The release safety case needs reviewed operating limits, bounded fault response,
independent witnesses and residual-risk acceptance by the flight/safety lead.
Missing hardware, CONOPS detail or evidence keeps the relevant gate open.

## Existing hazards and mitigation coverage

| ID | Hazard | Implemented mitigation and evidence limit |
|---|---|---|
| H-01 | Excessive or wrong-axis velocity | C++ finite/clamp/frame checks; verify each axis through ROS to wire independently |
| H-02 | Stale/unhealthy VIO motion | C++ VIO/watchdog and ROS source/receipt-age gates; no real estimator/fusion qualification |
| H-03 | Link loss or shutdown while commanding | Heartbeat/watchdog/zero attempts and loopback tests; no guarantee across a severed link |
| H-04 | Motion after mode change/disarm | Copter mode/armed watchdog gates; aircraft-class support and authority handover incomplete |
| H-05 | Fence breach | Target polygon validation, upload/readback and enable check; not continuous trajectory/traffic/altitude containment |
| H-06 | Unintended payload action | Dedicated consuming release interlock and off attempt; raw outputs bypass it; hardware timeout/feedback open |
| H-07 | Failsafe suppression | Narrow command surface and structural scans; generic user commands/parameter writers require semantic review |
| H-08 | Unauthorized commands | Local CLI nonempty-key opt-in and admission audit; no authenticated remote boundary or inherited library audit |

Historical Copter SITL results support development but do not close current
release gates without artifact provenance. The zero-delivery loopback test keeps
the outbound path open when inbound heartbeat stops; it does not simulate total
bidirectional radio failure. A lost physical link requires an independent
autopilot timeout/failsafe and operator procedure.

## Stable safety requirements

These retain their original obligations; partial coverage is not satisfaction.

| ID | Requirement | Current evidence / open scope |
|---|---|---|
| SR-VEL-01 | Clamp XY velocity to reviewed limits | C++ tests; per-axis limits, not a proven total horizontal-speed bound |
| SR-VEL-02 | Clamp vertical and yaw-rate velocity to reviewed limits | C++ tests; qualify per aircraft/profile |
| SR-VEL-03 | Reject the complete command for any non-finite component | C++ tests |
| SR-VEL-04 | Convert input and MAVLink frames explicitly and correctly | Core wire tests; end-to-end ROS/frame review open |
| SR-VEL-05 | Guided velocity requires armed state and GUIDED mode | Copter tests; never apply its numeric mode to Plane |
| SR-VEL-06 | Filter heartbeat to the commanded vehicle | Codec/UDP tests; authenticated source/target selection remains a security gate |
| SR-VIO-01 | Reject unhealthy, low-confidence, stale or unexpected-source VIO | Core/ROS tests; source timestamp, real sensor and fusion gates open |
| SR-VIO-02 | Stale VIO stops active velocity within watchdog interval | Deterministic tests; live estimator and full-load deadlines open |
| SR-LNK-01 | Commands require fresh FC heartbeat | Velocity gate/transport behavior tested; audit every discrete command path at G2 |
| SR-LNK-02 | Missing velocity input triggers a zero command within timeout | Watchdog tests; independent wire and FC observations required |
| SR-LNK-03 | Shutdown sends zero before closing an active link | Loopback ordering tests; live SITL and physical link evidence separate |
| SR-LNK-04 | Announce a standard GCS heartbeat for heartbeat-gated relays | Codec/UDP tests; historical Linux SITL report; requalify MAVSDK behavior |
| SR-FEN-01 | Upload, enable and verify FC fence before autonomous flight | Upload/readback/enable-reading tests; global preflight enforcement and all fence fields open |
| SR-FEN-02 | Reject position targets outside configured boundary | C++ target tests; live containment and full mission/velocity paths open |
| SR-PAY-01 | Validate servo channel and PWM before actuation | C++ generic range tests; board map and reserved payload channels open |
| SR-PAY-02 | Bound payload duration and de-energize outputs on failure | Dedicated release/off-failure tests; generic relay and physical power-loss behavior open |
| SR-PAY-03 | Release requires explicit operator interlock | Dedicated core release and UI tests; all raw output access must share authorization |
| SR-SEC-01 | No NOMAD command disables FC failsafes | Structural scan only; semantic allowlist and plugin parameter audit open |
| SR-SEC-02 | Authenticate command clients at trust boundary | Nonempty environment value is not authentication; production gate open |
| SR-SEC-03 | Authenticate and audit command requests | CLI admission logs only; final outcomes and all client/library entrypoints open |

## Additional hazards and proposed obligations

The IDs below reserve new obligations for the target architecture. They are
proposed engineering requirements (R), not CONOPS rules. Add real code/test
mappings when implemented; do not invent entries in the existing checked block.

| Hazard | Proposed requirement | Mitigation / objective falsification test | Gate |
|---|---|---|---|
| H-09 Conflicting writers | SR-AUT-01: one active owner and explicit handover | Concurrent CLI/ROS/plugin commands, pilot takeover and reconnect cannot resume old action | G2 |
| H-10 Stale position with fresh heartbeat | SR-TEL-01: per-field age and clock validity | Freeze position while heartbeats flow; position-dependent actions fail closed | G2 |
| H-11 Collision or missed traffic | SR-AIR-01: unknown/stale traffic never means clear | Crossing/head-on/reordered/expired tracks yield expected advisories with measured warning time | G4 |
| H-12 Wrong aircraft mode/transition | SR-TYP-01: validate aircraft class and state | Copter mode constants refused for Plane; failed/aborted VTOL transitions use reviewed response | G2/G7 |
| H-13 False identity or geolocation | SR-OBS-01: decisions retain evidence and uncertainty | Duplicate animals, occlusion, wrong datum and stale images cannot create unreviewed task actions | G5/G6 |
| H-14 Tracker confusion / duplicate payload | SR-TSK-01: bind task/target/action identity and expiry | Wrong tracker, reconnect or battery swap cannot retag/resample from replay | G6 |
| H-15 Payload jam/contact/power loss | SR-PAY-04: safe physical state and verified outcome | Interrupt power/link/feedback during actuation; independent measurement proves timeout and containment | G6/G7 |
| H-16 Mass, energy or navigation deficit | SR-OPS-01: qualify aircraft configuration and reserve | All-up weighing, endurance/transition energy and GNSS/RTK-loss evidence | G7 |
| H-17 Network/compute overload | SR-RES-01: bounded queues and safety execution | Video/server flood, dead worker, thermal throttling and disk-full cannot starve command deadlines | G3/G8 |
| H-18 Untrusted messages/replay | SR-SEC-04: authenticate, authorize and reject replay | Wrong/expired credentials, old session and malformed server/DDS/IPC input are refused and audited | G2/G8 |

Traffic advisories and explicit payload authorization are the initial user-selected
scope. Automatic conflict avoidance, approach, tagging and sampling remain TBD
after CONOPS review. Loss-of-traffic response and numeric limits need D07/D08;
do not silently choose hold/RTL/descent as an assessment rule.

## Required evidence by boundary

- Core: known state, one action, independent expected result; invalid inputs,
  boundaries, timeout, cancellation and failure paths.
- Transport: negative ACK, missing/wrong/duplicate ACK, wrong aircraft, stale
  fields, loss/reorder, actual stop wire delivery and independent FC outcome.
- ROS/perception: acquisition versus receive time, clock skew, reset counters,
  frame axes, delayed/replayed data, callback starvation and process failure.
- Payload: core permission plus physical timeout and attachment/sample evidence;
  ACK alone never proves delivery or collection.
- Flight: approved aircraft-specific procedure, independent pilot control,
  measured margins and recorded recovery. Fixed-wing and VTOL phases differ.
- Security: actual identity/authorization and outcome records at each boundary;
  a logged label saying auth=api-key is not proof of authentication.

The current traceability checker only resolves symbol/test names. It does not
test mapping completeness, semantic correctness, execution, or hazard coverage.
Missing coverage remains visible in the tables above.

## Existing machine-checked implementation mappings

Retained from the migration to avoid losing stable code/test references.
These mappings identify evidence locations; they do not assert full requirement
closure. Remap them in the MAVSDK cutover while retaining equivalent fault proof.

```cpp_traceability
SR-VEL-01 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-01 | src/safety/velocity_config.cpp:load_velocity_limits | tests/velocity_config_test.cpp::test_configured_limits_are_loaded
SR-VEL-02 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_accepts_clamped_frd_command
SR-VEL-02 | src/safety/velocity_config.cpp:load_velocity_limits | tests/velocity_config_test.cpp::test_configured_limits_are_loaded
SR-VEL-03 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-04 | src/mavlink/protocol.cpp:encode_velocity_setpoint | tests/core_test.cpp::test_velocity_frame_uses_expected_wire_layout
SR-VEL-05 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VEL-06 | src/mavlink/protocol.cpp:accepts_heartbeat | tests/core_test.cpp::test_heartbeat_filter_accepts_vehicle_only
SR-VIO-01 | src/safety/velocity.cpp:evaluate_velocity | tests/safety_test.cpp::test_safety_velocity_rejects_each_fault
SR-VIO-01 | src/safety/vio_source.cpp:VioSourceValidator::validate | tests/vio_source_test.cpp::test_vio_source_validator_rejects_wrong_source
SR-VIO-01 | ros2/nomad_ros/src/node.cpp:on_velocity_command | tests/ros/test_nomad_ros_integration.py::test_vio_source_gate_blocks_mismatch
SR-VIO-02 | src/safety/watchdog.cpp:evaluate_watchdog | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_stale_vio_and_mode_loss
SR-LNK-01 | src/vehicle/vehicle.cpp:set_velocity | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_link_loss
SR-LNK-02 | src/safety/watchdog.cpp:evaluate_watchdog | tests/safety_test.cpp::test_vehicle_watchdog_stops_for_command_timeout
SR-LNK-03 | src/mavlink/udp_connection.cpp:send_velocity | tests/safety_test.cpp::test_vehicle_stop_velocity_sends_zero
SR-LNK-04 | src/mavlink/protocol.cpp:encode_gcs_heartbeat | tests/codec_golden_test.cpp::test_gcs_heartbeat_encoder_matches_mavlink_reference
SR-LNK-04 | src/mavlink/udp_connection.cpp:send_gcs_heartbeat_locked | tests/udp_connection_test.cpp::test_unlatched_connection_sends_gcs_heartbeats
SR-LNK-04 | src/mavlink/udp_connection.cpp:announcement_override | tests/udp_connection_test.cpp::test_relay_address_override_targets_prelatch_announcements
SR-FEN-01 | src/vehicle/vehicle.cpp:upload_fence | tests/safety_test.cpp::test_vehicle_upload_fence_validates_boundary
SR-FEN-01 | src/vehicle/vehicle.cpp:verify_fence_uploaded | tests/safety_test.cpp::test_vehicle_verifies_fence_status_and_fails_closed
SR-FEN-01 | src/mavlink/fence.cpp:upload_fence_plan | tests/safety_test.cpp::test_vehicle_upload_fence_rejects_transport_failure
SR-FEN-01 | src/mavlink/fence.cpp:download_fence_plan | tests/safety_test.cpp::test_vehicle_verifies_fence_status_and_fails_closed
SR-FEN-01 | src/mavlink/params.cpp:read_param | tests/safety_test.cpp::test_vehicle_verifies_fence_status_and_fails_closed
SR-FEN-02 | src/safety/geofence.cpp:evaluate_global_position | tests/safety_test.cpp::test_vehicle_fence_rejects_target_before_transmission
SR-FEN-02 | src/safety/geofence.cpp:evaluate_position | tests/fence_config_test.cpp::test_local_polygon_with_nonfinite_vertex_fails_closed
SR-PAY-01 | src/safety/payload.cpp:validate_servo_command | tests/safety_test.cpp::test_vehicle_payload_commands_require_interlock_and_validate_ranges
SR-PAY-02 | src/safety/payload.cpp:clamp_release_duration | tests/safety_test.cpp::test_payload_validation_and_interlock
SR-PAY-02 | src/vehicle/vehicle.cpp:release_payload | tests/safety_test.cpp::test_vehicle_payload_on_failure_still_attempts_off
SR-PAY-02 | src/vehicle/vehicle.cpp:release_payload | tests/safety_test.cpp::test_vehicle_payload_off_failure_is_reported
SR-PAY-03 | src/safety/payload.cpp:ReleaseInterlock::evaluate_release | tests/safety_test.cpp::test_payload_validation_and_interlock
SR-SEC-01 | src/vehicle/vehicle.cpp:send_command | tests/core_test.cpp::test_command_frame_has_expected_header
SR-SEC-01 | src/main.cpp:run_command | tests/test_cpp_command_surface.py::test_cpp_command_surface_has_no_failsafe_controls
SR-SEC-02 | src/main.cpp:run_command | tests/test_core_client_contract.py::test_every_actuation_verb_refused_without_key_before_any_socket_work
SR-SEC-03 | src/main.cpp:audit_command | tests/test_core_client_contract.py::test_actuation_with_key_reaches_transport_and_audits
```

## Operational release rule

No flight follows a documentation-only assumption. Before an authorized hardware
session, establish the correct aircraft/profile/firmware, safe physical setup,
current state and command owner, required capabilities and reviewed limits.
Verify autopilot failsafes and manual takeover independently.

Read back the fence, output mapping and required navigation state; invalidate
payload permissions before maintenance, battery swap or restart. On uncertainty,
report failed/unknown outcome and follow the reviewed procedure. After the
session, observe safe payload state, disarmed aircraft and restored configuration.
Full gate evidence and role ownership are in [migration](migration.md).
