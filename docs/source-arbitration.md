# Command sources and takeover qualification

NOMAD admission arbitrates typed requests to one `nomad-runtime`. It does not
arbitrate the whole aircraft. ArduPilot accepts or rejects traffic according to
its firmware, parameters, input state and mode. A pilot or external GCS mode
change does **not** automatically revoke NOMAD output authority. Operators must
explicitly revoke it; a controller-side pilot-priority interlock remains an
unqualified requirement. Do not assume that changing mode blocks a later servo
command.

This qualification slice starts from NOMAD
`34d93335c41a000d78a323436e9027704bffc160` (after PR #52). It adds evidence,
not production termination, a C2-loss policy, or another command owner.

## Intended sources

This page also records the separate physical arbitration slice based on merged
PR #53, at `8e9c1450b433798aaa8d641eb627ea7091cfccc8`. Its read-only physical
observations do not extend the earlier SITL qualification claims.

| Source | Path | Can command? | NOMAD controls it? | Qualification status |
|---|---|---|---|---|
| `nomad-runtime` | runtime → MAVSDK → standalone ground router → aircraft router → FC | Yes, supported typed requests | Yes, this runtime's request admission and tested command-send fence | Software tests; disarmed pinned SITL scenario below |
| Mission Planner plugin typed requests | MP plugin → local runtime IPC | Yes, through runtime | Yes, same software boundary | Software-qualified request boundary; no independent vehicle fallback |
| Mission Planner native vehicle connection | Native/direct GCS path → FC | Potentially | No | External authority; bench interaction open |
| RC / ELRS pilot | Receiver → FC input | Yes, subject to configured mode/input handling | No | Production input/map unresolved; physical qualification required |
| Maintenance / independent MAVLink source | Separate source → router or FC | Potentially | No | External authority; isolated source-250 SITL mode test only |
| Qualification tooling | Isolated direct simulator/peer path | Test only | Separate from production admission | Non-installed direct driver; `NOMAD_INTEGRATED_FLIGHT=1` inhibits its actuation, but is not an aircraft access-control boundary |
| ROS 2 observer | Receive-only telemetry endpoint | No supported commands | Observation boundary only | Software-tested; not a control source |

The ground router's `mission_planner` consumer is receive-only for aircraft
command egress. This restriction is not FC authentication, does not block a
separate native MP connection, and does not apply to every aircraft-side
`mavlink-router` endpoint. Source system/component IDs identify MAVLink packets;
they are not a cryptographic grant of authority. See
[architecture](architecture.md), [runtime IPC](runtime-ipc.md), and the
[ground-router contract](../infra/transport/ground_router/README.md).

## Qualification levels and checks

| Check | Evidence level | Authoritative observation | Boundary it does not prove |
|---|---|---|---|
| Startup, admission, expiry/sequence, cache eviction, session and restart unit cases | Software test | Runtime response, fake transport calls and state | FC acceptance or physical action |
| Production-runtime retry/revoke/session fixture | Deterministic peer | Independently counted UDP commands; suppressed retry after revoke | ArduPilot arbitration |
| Queued `COMMAND_LONG` / `COMMAND_INT` final-send probe | Deterministic peer | UDP socket receives no old admitted work after fence transition | Every MAVLink message or other transports; queue proof uses a probe gate |
| Disarmed Copter runtime channel-5 command | SITL controller behavior | Runtime result plus fresh FC `SERVO_OUTPUT_RAW` change | QuadPlane v1 mutation support, physical servo movement, flight command support or payload safety |
| Independent source-250 mode request while NOMAD owns/is revoked | SITL controller behavior; simulated external-source takeover | Fresh FC heartbeat mode transition | RC/ELRS pilot priority, simultaneous conflict policy or physical takeover |
| NOMAD-path pause/recovery and runtime restart | Software plus SITL controller behavior | New session/incarnation, no owner, rejected old context, no command frames | Production C2-loss response, redundant radio failover or FC restart |
| `SIM_RC_FAIL` injection/restoration | SITL controller behavior | Disarmed receiver-health loss/recovery in `SYS_STATUS` | Physical receiver loss, aircraft failsafe action or pilot takeover |
| Approved transmitter/receiver input and source conflicts | Controller bench qualification | FC RC input, mode, command acceptance, measured output and timing | Airborne dynamics |
| Takeover in hover, fixed wing and transitions | Physical flight qualification | Independent mode/output/trajectory and pilot-control evidence | Untested aircraft, firmware, phases or faults |

The live runtime authority scenario uses the existing Copter-4.7.1 `quad`
stack, verified at firmware SHA
`dbe792162d06cab66c3475fd5556bf7a120f119e`, with the unchanged
[fence](../docker/sitl-fence.parm) and [stream](../docker/sitl-streams.parm)
files. It requires a dedicated native Docker simulator before sending anything
and reads back `SERVO5_FUNCTION=0`. Channel 5 here is a disabled simulator
output, **not** an approved production RC/payload channel. Protocol v1 has no
flight-mode request, so its runtime operation is an output change. The initial
disabled output can be zero; runtime PWM validation is preserved. After NOMAD
shutdown the isolated test GCS restores that initial output and observes fresh
FC readback. See the
[scenario procedure](../tests/sitl/README.md#runtime-authority-and-independent-source).

The attempted pinned QuadPlane runtime output scenario exposed an existing
capability limit: [`supports_operation`](../src/vehicle/operation.cpp) rejects
servo/relay/motor-test/gimbal requests for QuadPlane. Consequently **none of the
v1 runtime mutations is executable on QuadPlane**. Admission alone does not
qualify an operation. The capability gate remains intact; successful Copter
runtime-output evidence cannot be promoted to QuadPlane. Its pinned
[`quadplane-tilttri.parm`](../docker/quadplane-tilttri.parm) and existing
independent mode/RC-fault scenarios establish separate controller evidence.
Qualifying a future typed QuadPlane runtime operation remains open.

`handback` means explicitly returning software command authority **to NOMAD**
after revocation. `revoke` releases that authority. Neither operation switches
an FC mode, establishes pilot control, selects a receiver or silences an
external MAVLink source. Revoke prevents later covered sends; it does not undo
an already accepted FC output command or reset a latched output. Record the
actual output after revoke and select its approved safe restoration procedure.
After an admitted runtime loses a session, `admit`
cannot substitute for `handback`; a newly restarted inhibited runtime instead
requires a new `admit`. Link restoration alone does neither.

## Pinned controller mechanisms

The table is an audit of the pinned ArduPlane source, not a production parameter
prescription. Read back the actual controller's complete parameter set for
every evidence record. Historical `SYSID_*` names must not replace the pinned
4.7.1 `MAV_*` names.

| Mechanism / parameters | Pinned behavior and qualification implication |
|---|---|
| `MAV_SYSID`, `MAV_GCS_SYSID`, `MAV_GCS_SYSID_HI`, `MAV_OPTIONS` | FC/source identities and allowed GCS range. Source defaults have primary GCS 255, high range 0, enforcement off (`MAV_OPTIONS=0`). With enforcement off, ordinary packets from other system IDs may pass; same-system and selected GCS IDs are accepted by the source check. This is acceptance filtering, not exclusive control. |
| Runtime source identity | Pinned MAVSDK GroundStation defaults are system 245/component 190. This differs from default primary GCS 255. An enforced GCS-ID profile must deliberately admit intended source IDs; never change it merely to make a test pass. |
| `SET_MODE`, `COMMAND_LONG`, `COMMAND_INT` | Packet/source checks, target addressing and command/mode validation determine acceptance. `SET_MODE` has no separate primary-GCS check in its handler. NOMAD's software generation is not transmitted as an FC ownership grant. |
| MAVLink signing, `MAVn_OPTIONS` and serial/UDP routes | Per-channel `MAVn_OPTIONS` defaults to 0; bits 0/1/2/3 permit unsigned MAVLink2 / disable forwarding / ignore streamrate / forward bad-CRC packets. This differs from global source-ID enforcement. Signing configuration and endpoint reachability require hardware records; transport priority is not sender arbitration. |
| `SERIALn_PROTOCOL`, `SERIALn_BAUD`, `SERIALn_OPTIONS` | Controller link/input-port selection and serial behavior. Protocol 23 is a possible serial RCIN route. Forwarding/streamrate options moved from serial flags to `MAVn_OPTIONS` in 4.7. Record the actual CRSF/ELRS receiver mode, port/wiring and readbacks; no port configuration is approved here. |
| `RC_CHANNELS_OVERRIDE`, `MANUAL_CONTROL` | Handlers require a selected GCS system ID, unlike ordinary mode traffic with default enforcement off. Runtime v1 has no override/manual-control request. Audit maintenance/native tools for such traffic. |
| `RC_OVERRIDE_TIME`, `RC_OPTIONS`, override-enable aux option 46 | Default override expiry is 3 s; 0 disables overrides, -1 prevents expiry. `RC_OPTIONS` bits 0/1 ignore receiver/overrides; bit 2 ignores the receiver failsafe bit; bit 10 enables multiple receivers; bit 13 selects the ELRS 420 kbaud option. Override enable state also gates acceptance; live overrides precede receiver data. |
| `RC_PROTOCOLS`, `RCMAP_*`, per-channel calibration and `RCn_OPTION` | Protocol selection, axis mapping/calibration and aux functions control FC input interpretation. `RC_PROTOCOLS=1` enables all; bit 9 selects CRSF. Axis defaults 1/2/3/4 are not an approved production ELRS map. Physical wiring/protocol must be measured. |
| `FLTMODE_CH`, `FLTMODE1..6`, `RCn_OPTION` | Switch input and configured mode slots/aux functions select modes. An external mode transition is not a runtime revoke or automatic physical handback. |
| `STICK_MIXING` | Pilot input can be mixed into automatic modes without changing mode; pinned Plane default is FBW stick mixing. Value 3 has QuadPlane yaw-only VTOL behavior. Actual control response requires bench/flight measurement. |
| `THR_FAILSAFE`, `THR_FS_VALUE`, `RC_FS_TIMEOUT` | Receiver/throttle loss handling; source defaults include enabled throttle failsafe, threshold 950 and 1 s RC timeout. `THR_FAILSAFE=2` ignores failed RC input without triggering the RC failsafe action. Real receiver hold/no-pulse/failsafe-bit behavior requires bench evidence. |
| `FS_SHORT_ACTN`, `FS_LONG_ACTN`, `FS_LONG_TIMEOUT`, `FS_GCS_ENABL` | Short/long loss actions and GCS monitoring. Source long timeout is 5 s and GCS failsafe is disabled by default. Configured GCS heartbeat monitoring does not imply every external source or NOMAD (default ID 245) is monitored. Long failsafe clears RC overrides; action and recovery still depend on mode/reason/profile. |
| `Q_OPTIONS` bits 5/20; current mode and failsafe reason | QuadPlane RC-loss selection can use QRTL/RTL instead of QLAND. In radio-failsafe recovery, saved entry mode can be restored when the control-mode reason remains radio failsafe. This is FC recovery, not NOMAD admission. |
| `Q_ENABLE`, `Q_TRANS_FAIL`, `Q_TRANS_FAIL_ACT` | Pinned SITL classification/transition profile (`Q_ENABLE=2`); transition-failure parameters belong to phase-specific qualification, not source arbitration. This PR does not change them or establish a production C2 response. |
| `SIM_RC_FAIL` | Native simulator receiver fault injection only. Existing disarmed probe proves health loss/restoration; not physical RF failure or flight response. |

These failsafe names/actions describe **ArduPlane**. The Copter authority test
records its separate `FS_GCS_ENABLE` readback and does not qualify a failsafe
action; do not apply Plane's `FS_GCS_ENABL` or short/long action rules to Copter.

Pinned implementation references:
[GCS parameters](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS.cpp#L38-L66),
[source acceptance](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L7180-L7201),
[mode handler](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L2878-L2904),
[override/manual-control handlers](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L4177-L4225),
[RC parameters](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channels_VarInfo.h#L84-L113),
[RC input selection](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channel.cpp#L303-L310),
[Plane failsafe parameters](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/Parameters.cpp#L406-L510),
and [radio recovery](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/events.cpp#L243-L251).

The pinned `FS_GCS_ENABL` description specifies monitoring after a first primary
GCS heartbeat: value 1 monitors heartbeat, 2 also monitors `RADIO_STATUS.remrssi`,
and 3 monitors heartbeat only in AUTO. Check actual heartbeat identity/range and
loss behavior; default runtime ID 245 is outside the default GCS ID 255.
Additional pinned references are
[channel options](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp#L200-L216)
and [serial configuration](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_SerialManager/AP_SerialManager.cpp#L185-L281).

## Required production RC / ELRS inputs

No approved production channel map or controller input-priority profile is
recorded. The [PRD](prd.md) mentions "CH5 currently arms" but requires verifying
actual mode/aux mapping; this is an unverified project assumption, not approval.
SITL defaults and deployment transport labels are not approval.
Keep this table unresolved until the aircraft owner supplies a reviewed record.

| Required decision | Required retained value and measurement | Current status |
|---|---|---|
| Executable runtime operation | Exact aircraft class, typed request and capability evidence before bench step 4 | Copter output tested in SITL; no executable QuadPlane v1 mutation |
| Receiver and FC wiring | Receiver model/firmware, ELRS settings, output protocol, wiring, FC port and protocol selection | Required; not qualified |
| Input calibration/map | Every control/aux channel, `RCMAP_*`, min/trim/max/reversal/deadzone; measured FC channel values | Required; do not derive from SITL |
| Pilot mode/takeover switch | Physical switch positions, `FLTMODE_CH`, `FLTMODE1..6`, `RCn_OPTION`, debounce and observed mode/time | Required; no approved production allocation |
| Override policy | Whether any source may emit RC overrides; allowed sender, enable option, timeout, release and physical-input precedence | Required; runtime has no RC-override request |
| GCS acceptance/identity | Runtime/native GCS/maintenance IDs, enforcement flags, signing policy and collision behavior | Required; IDs alone do not authenticate |
| Receiver-loss behavior | Receiver no-pulses/failsafe-bit/held-value behavior, FC timeout, short/long actions in each phase | Required; measure actual loss, not only disconnect a nominal link |
| GCS / total C2 loss | Selected monitored sources, timeout, action, redundant links and recovery/latching decision | Separate open C2 policy; not closed here |
| Pilot priority and handback | Approved response to simultaneous requests, explicit runtime revoke, physical takeover and re-admission conditions | Bench and flight qualification open |

## Controller bench and aircraft procedure

This is a procedure to run later, **not a passed qualification record**. A flight
and safety lead must approve the real configuration, expected transitions,
timing bounds and safe test outputs before execution. Begin with propellers
removed and power/output isolation verified. Measure output electrically or
with an approved unloaded actuator; do not use the simulator's channel 5 on an
aircraft without verifying its actual function. No automatic termination or
failsafe-disabling recipe is authorized here.

Use one test record per aircraft/profile/phase. For each row capture UTC and
monotonic transition time, active sources, runtime owner/generation/session
relationship, received RC channels, requested/observed FC mode, armed state,
ACK result, actual output, latency, expected outcome and actual outcome. An ACK
alone cannot pass an output or takeover row. Define expected outcomes and
maximum takeover/timeout latency **before** running the test. Any unexpected
source acceptance, continued old command, stale state, failure to recover, or
missing evidence fails the row and stops escalation to powered/flight tests.

| Step | Stimulus | Required observation / acceptance |
|---|---|---|
| 1. Inhibited startup | Start FC, router and runtime without admission | No runtime owner; typed mutation rejected; no corresponding command/output change |
| 2. Pilot input | Exercise approved physical transmitter controls and mode switch | Correct measured channels, FC mode and safe outputs; record every switch position |
| 3. Admit NOMAD | Explicit admission under approved safe preconditions | New owner/generation; pilot path remains available; no unsolicited output |
| 4. NOMAD command | Issue one approved bounded typed command | Delivery, FC acceptance and independently measured output agree; report failure separately |
| 5. Pilot takeover while NOMAD active | Operate takeover switch/control during a bounded NOMAD operation | Approved pilot mode/output wins within bound; determine whether continuing NOMAD output is accepted; do not infer runtime revocation from mode change |
| 6. Runtime revoke | Revoke with work queued/retrying; attempt fresh and old requests | No owner; fresh/old requests rejected; no covered command after completed fence transition; independently confirm pilot/native control |
| 7. Explicit handback | Confirm approved pilot release conditions, then runtime `handback` | New generation, only new request context executes; verify FC mode/input/output separately |
| 8. NOMAD link loss | Remove the selected NOMAD transport while independent pilot observer remains | Runtime inhibition and measured FC response/time; do not equate this with total C2 loss |
| 9. RC link loss | Remove physical RC RF path using approved isolation | Receiver/FC loss indication, approved short/long failsafe state/output/time; restore and measure recovery |
| 10. NOMAD reconnect | Restore transport; separately restart runtime | Recovery alone leaves NOMAD inhibited; old context rejected; explicit handback or new-start admission required |
| 11. Native GCS | Issue approved native MP action while NOMAD inhibited, then admitted | Record actual FC acceptance/mode/output; verify receive-only router consumer restriction separately |
| 12. Conflicting sources | Exercise approved bounded RC/native GCS/NOMAD/maintenance conflict pairs and order reversals | Observed winner, latency and output match preapproved matrix; IDs do not create an assumed priority |
| 13. FC restart | Safely restart isolated FC with links present | Safe boot outputs/input, fresh session, NOMAD inhibited, old context rejected; explicit recovery procedure |
| 14. Repeat in required phases | Only after bench pass and separate flight authorization, repeat approved cases in hover, fixed wing and transitions | Independent trajectory/control evidence within approved envelope; label physical-flight results separately |

Retain an immutable evidence bundle containing exact NOMAD SHA and build,
ArduPilot firmware SHA/version and FC build/board identity, complete before/after
parameter files, approved RC channel map, transmitter/receiver/ELRS firmware and
configuration excluding secrets, wiring/input protocol, both router configurations, system and
component IDs for every source, synchronized timestamp method, MAVLink tlogs,
FC DataFlash logs, runtime logs, observer measurements/video, initial state,
every expected/actual result, failed attempts and restoration results. Keep
signing-key material and ELRS binding secrets out of all retained logs; retain
only signing configuration/key presence or a nonsecret reference. Keep secrets
and machine-specific deployment identifiers out of this repository;
link a reviewed restricted evidence record instead. Record operator and safety
review approval, deviations and any residual limitation.

A controller bench pass still leaves RF independence, physical pilot handling,
airborne output/trajectory and phase-dependent takeover open. SITL must never
mark this procedure passed. Qualification records belong in
[qualification status](qualification.md); unresolved work stays in
[TODO](../TODO.md).

## Physical bench observation 2026-09-30

This separate slice starts at merged PR #53, code
`8e9c1450b433798aaa8d641eb627ea7091cfccc8`. Operator confirmed disarmed,
props removed, Mission Planner closed, no USB joystick and no LTE hardware.
Read-only ELRS USB observation used 460800 baud with DTR/RTS low. Initial
capture sent zero bytes; subsequent sends were parameter read requests and
`MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)` only, source 254/component 190.
No parameter writes, PC/observer-generated overrides, manual-control,
arm/mode/actuator commands, RF faults or runtime/router deployment were performed.
The handset's ELRS-originated override stream was observed.

The FC reports ArduPlane **4.7.1 stable**, custom-version ASCII `dbe79216`,
fixed-wing heartbeat type 1, autopilot 3, system/component 1/1 and `Q_ENABLE=1`.
`Plane-4.7.1` resolves to `dbe792162d06cab66c3475fd5556bf7a120f119e`, matching
the reported prefix. The eight-byte prefix is not a full flashed-image attestation.
Operator reports RadioMaster/EdgeTX handset, NOMAD module, RP2 receiver and
ELRS 4.0.0 with MAVLink serial output, **no separate CRSF/native RC input**.
Exact EdgeTX build, flashed ELRS hashes and receiver-generated source IDs remain
unrecorded; DBR4 was not exercised.

The indexed pre-change snapshot has 1202/1202 unique parameters, SHA-256
`a2fb42e188c9bb4a457cc5bd35ee2f01b0aba51722c354bc39e0e2512f185519`.
Full snapshots/capture summaries remain in ignored local
`build/hardware-observation/`. Do not commit raw telemetry, location, UID,
binding/signing secrets or host-specific port names.

| Parameter | Actual readback |
|---|---|
| `RCMAP_ROLL/PITCH/THROTTLE/YAW` | 1 / 2 / 3 / 4 |
| `FLTMODE_CH` | 8; physical switch identification pending |
| Every `RC1_OPTION` through `RC16_OPTION` | 0; no Aux 46 configured |
| `RC_OPTIONS`, `RC_OVERRIDE_TIME` | 32 / 3 seconds (readback, not measured expiry) |
| `MAV_GCS_SYSID`, `MAV_GCS_SYSID_HI`, `MAV_OPTIONS` | 255 / 0 / 0 |
| `RC_FS_TIMEOUT`, `THR_FAILSAFE`, `THR_FS_VALUE`, `FS_GCS_ENABL` | 1 second / 1 / 950 / 0 |
| `SERIAL1_PROTOCOL`, `SERIAL1_BAUD` | 2 (MAVLink2) / 460 |

All six `FLTMODE1..6` slots read back 17 (QSTABILIZE); no configured slot
variation establishes a usable physical mode-selection policy. Stick calibration
is still MIN/TRIM/MAX 1100/1500/1900, REVERSED 0, DZ 30 on channels 1–4;
observed endpoints exceed these configured limits. No calibration was changed.

| Physical action | Effective channel/range observed | Evidence limit |
|---|---|---|
| Roll retry | CH1 989–2010; last 2010 | CH2 also moved 1355–1568; not fully isolated |
| Pitch retry | CH2 989–2012; centred 1500 | 90 samples, 0.999 Hz; native input not proved |
| Throttle | CH3 989–2006; returned 989 | 60 samples, 1.001 Hz; native input not proved |
| Yaw attempts | CH4 fixed 1495 | No excursion captured; physical identification incomplete |
| Switch movement during yaw retry | CH9 1000–1500, CH10 1500–2000 | Operator identities unconfirmed; not a source-selector mapping |
| Flight-mode switch / source-selector LOW/MID/HIGH | NOT MEASURED | No channel assignment or mix change proposed |

Configured roll/pitch normalization clips the measured endpoints to -1/+1
using the saved limits; centres within DZ map to zero. Configured throttle
normalization clips 989/2006 to 0/1. These are mathematical interpretations of
current calibration, not hardware axis-rate measurements. Telemetry observation
rate near 1 Hz is not the handset/RF/receiver update rate. ELRS source schedules
handset override frames every 10 ms; actual receiver output timing was not tapped.

The after snapshot also contains 1202/1202 parameters, SHA-256
`516fd67473d9829e72e69ca7c95929538aabcdf7cd54742b10d5ca37efc53a32`.
The sole before/after difference is automatic `STAT_RUNTIME` 4864→6011;
every other parameter value/type is equal. No configuration restoration was
necessary because no parameter write occurred. Final snapshot observed disarmed.

### Blocking physical-path finding

**Do not configure the paired Aux-46 selector on this MAVLink-only topology.**
ELRS 4.0.0 release source `ed9fc3e637207e8d656ffe9b1b3e8eef418573c6`
converts handset channels to MAVLink overrides, while independently forwarding
PC MAVLink traffic. Its receiver does not arbitrate pilot/computer ownership.
See [ELRS sendRCFrame and forwarding](https://github.com/ExpressLRS/ExpressLRS/blob/ed9fc3e637207e8d656ffe9b1b3e8eef418573c6/src/src/rx-serial/SerialMavlink.cpp#L26-L58)
and [official topology](https://www.expresslrs.org/software/mavlink/).
Source defaults are 255/component TELEMETRY_RADIO (68), target 1/component ALL;
Lua can change IDs, so these are source expectations, not observed packet IDs.

Actual stick excursions reached effective RC telemetry at about 1 Hz, but
`chancount=0`, RSSI 254 and SYS_STATUS receiver healthy remained unchanged.
This is consistent with MAVLink pilot input and does not establish native HAL
RC input or RF freshness. The handset path is independent of PC/runtime, but
shares ArduPilot's override input domain with computer control.

On the matching FC source, assigning Aux 46 makes that channel reject
`set_override`, including handset-generated overrides. Disabling overrides
would disable this pilot-input form too. Stop before writes or competing
override tests. Redesign must supply a demonstrated native RC input alongside
telemetry, or a separately reviewed FC-side arbiter with trusted physical
provenance. Receiver choice does not change runtime/router ownership. Do not
weaken the gate or restore arbitrary Mission Planner writes as a workaround.

Only after native RC ingress is proved, reconsider the paired physical switch:
A = measured three-state unassigned source request; B = Aux 46, LOW for PILOT
and HIGH for both JOYSTICK/AUTO. Source, gate, flight mode, arming and termination
are separate functions. All safety channels are excluded from software
overrides. Effective RC_CHANNELS alone cannot authenticate A or prove freshness.

### Installed-source audit and message decision

The matching ArduPilot release source verifies:

- Aux 46 LOW clears/disables, HIGH enables, MIDDLE retains state; its channel
  rejects overrides. Receiver loss need not deliver a new LOW. See
  [gate protection/expiry](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channel.cpp#L494-L533)
  and [Aux 46 switch](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channel.cpp#L1297-L1312).
- `RC_OPTIONS=32` is throttle arming check; bit14 `CLEAR_OVERRIDES_BY_RC` is absent.
  [Options and gate clearing](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channel.h#L616-L660).
- Override/manual handlers require the configured GCS sysid despite
  `MAV_OPTIONS=0`; component IDs do not authenticate control. Runtime's default
  245 differs from actual primary GCS 255.
  [Handlers](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L3893-L3942).
- Positive override timeout expires each channel; 0 disables; -1 prevents
  expiry. Fallback does not check receiver sample freshness.
  [Channel update](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/RC_Channel/RC_Channel.cpp#L290-L307).
- Receiver status can reflect previously received overrides and failsafe state,
  rather than a fresh physical sample.
  [RC status](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS.cpp#L576-L584).

Engineering decision: prefer `MANUAL_CONTROL` for a future qualified four-axis
HID stream on this Plane/QuadPlane release, **pending physical-ingress redesign
and final-send fencing**. Plane maps y→roll, reversed x→pitch, z→throttle and
r→yaw through RCMAP/calibration and cannot address selector/mode switches in
that handler. See [Plane mapping](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/GCS_MAVLink_Plane.cpp#L931-L940).
Send all four fresh axes; validate device, sequence, range and deadman; explicitly
revoke/release. Omitting axes is not a global release recipe. RC override can
address safety channels and uses different release sentinels for 1–8 versus
9–18. Neither message repairs stale fallback or creates pilot priority.
Aux 46 does not block arbitrary navigation/servo/relay MAVLink commands.
Production runtime exposes neither manual message today.

### Stale receiver and shared RF/LTE hazard

The installed shared channel code can fall back to stale receiver values after
partial overrides expire. Plane may refresh radio-valid timing on override
input with plausible retained throttle; see
[radio failsafe](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduPlane/radio.cpp#L172-L244).
`RC_OVERRIDE_TIME=3` alone therefore does not make surviving LTE overrides safe.
[Issue 32862](https://github.com/ArduPilot/ardupilot/issues/32862) reports stale
mode fallback on Copter; [issue 33125](https://github.com/ArduPilot/ardupilot/issues/33125)
reports a related partial override fault. Shared-source exposure is verified;
installed Plane hardware reproduction is **NOT RUN**. LTE is absent and the
native-pilot prerequisite failed. The earlier Aux-46 self-write issue is fixed
in installed source ([PR 33275](https://github.com/ArduPilot/ardupilot/pull/33275));
that protection does not create native RC input.

### Read-only mapper and qualification gates

`scripts/hardware/observe_rc.py --port <ELRS_USB_PORT> --snapshot --duration 60
--output build/hardware-observation/before.json` saves the full indexed snapshot.
Without `--snapshot`, it transmits no MAVLink packets. Capture one control at
a time with `--label <control>` and a new file. It refuses overwrite and aborts
on armed/stale ownship heartbeat. Save each switch position separately for
LOW/MID/HIGH ranges/jitter; travel min/max alone cannot establish MID. Normalize
axes against saved `RCn_MIN/TRIM/MAX/REVERSED`, not assumed endpoints.

After native-pilot redesign, declare latency bounds, safe unloaded output
isolation, witness and abort conditions before every override/takeover row.
Characterize MP bench-only IDs/message set/rate/stop behavior without enabling
normal router MP egress. Test continued overrides against PILOT, JOYSTICK axis
allowlist, AUTO admission and return to PILOT, reverse transitions, noisy/missing
selector, and old queued frames. No autonomous movement is part of this slice.
Take full after snapshots and compare every parameter/type; prove restoration
after any future edits. Missing evidence is ABORT/NOT RUN, never PASS.

| Scenario | This run | Required next evidence |
|---|---|---|
| ELRS + LTE healthy | NOT RUN: LTE absent | Actual dual-link operation |
| ELRS healthy, LTE lost | NOT RUN: LTE absent | Physical loss and ELRS continuity |
| ELRS MAVLink lost, pilot available | NOT RUN | Prove separable native pilot path |
| ELRS/pilot lost, LTE healthy | NOT RUN: LTE absent | Stale-input/failsafe reproduction |
| LTE joystick active, LTE lost | NOT RUN: HID/LTE absent | Expiry/release/no retained owner |
| ELRS returns after LTE control | NOT RUN | No historical selector/owner restoration |
| Both MAVLink paths lost | NOT RUN | Qualified FC response/runtime inhibition |
| Runtime restart | Policy software tests only | Real bench restart/no re-admission |
| Router restart | Policy software tests only | No buffered old commands on real path |
| Selector during link changes | Policy software tests only | Measured transitions during actual faults |

`ControlSourceGate` is not wired into runtime: synthetic tests prove policy only.
debt: policy-only until native pilot ingress is qualified; revisit when a trusted
physical selector/freshness feed is demonstrated; then connect observations and
per-frame final-send admission to runtime before exposing flight joystick IPC.
Runtime must serialize observations, invalidation and final per-frame send
under its authority lock, combining source generation with existing incarnation,
session and expiry. Future HID samples contain four axes, allowlisted device ID,
increasing sequence, local monotonic receive time and held deadman. Loss/restart
requires explicit fresh admission; route restoration cannot grant authority.
Router remains link selection/health/deduplication infrastructure. Existing CSV
bridge retains virtual axes after bad/missing frames and is not a flight ingress.

Exactly three GPT-6 Luna MAX read-only audits were used. ArduPilot/ELRS audit
found the incompatible MAVLink-only pilot ingress and stale-fallback exposure;
code audit found absent selector telemetry/manual IPC/final-send coverage and
peripheral-only joystick; safety review required measured provenance, declared
timing bounds, honest NOT RUN rows and complete snapshot/restoration evidence.
Findings are resolved by stopping unsupported mutations, preserving one writer,
and explicitly retaining physical-ingress and integration qualification gates.
