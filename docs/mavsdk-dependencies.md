# MAVSDK Phase A dependency inventory

This inventory records the optional Phase A build inputs at the reviewed NOMAD
gitlink. It is an engineering and notice audit, not a release approval. The
production core does not use MAVSDK yet.
The CONOPS v1.0 reconciliation did not change dependency pins or accepted evidence.
MAVSDK remains a project prerequisite, not an organizer-prescribed library.
Competition transport/traffic requirements do not justify adding speculative
network plugins: obtain the official server contract first. Re-audit this
inventory when production parity adds required MAVSDK plugins or changes build
options; the telemetry-only Phase A inventory cannot cover those future inputs.

| Component | Reviewed source | License found in fetched source |
|---|---|---|
| MAVSDK | NOMAD gitlink `9884f109533f564bc6250e5471e6301d3a62f4a7` | BSD-3-Clause |
| MAVSDK proto | nested gitlink `1fd0bc7a05c21336227b1eab266b8b610401cf38` | BSD-3-Clause |
| Asio | tag `asio-1-30-2` | Boost-1.0 |
| fmt | tag `12.1.0` | MIT |
| libevents | commit `840a88ea226d4eb0fd4c391ce860317422756435` | BSD-3-Clause |
| libmavlike | commit `90498b14262137ae10b633705810e81bdb85de9c` | BSD-3-Clause |
| MAVLink | commit `d6a7eeaf43319ce6da19a1973ca40180a4210643` | generator (L)GPL-3.0 with MIT output exception |
| nlohmann JSON | archive tag `v3.12.0`, SHA-256 `4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187` | MIT |
| PicoSHA2 | commit `1bf940d8a03bb752604fbb366d47b97b50b9e6ce` | MIT |
| tinyxml2 | tag `11.0.0` | Zlib |
| liblzma from XZ Utils | archive `5.4.5`, SHA-256 `135c90b934aee8fbc0d467de87a05cb70d627da36abe518c357a873709e5b7d6` | public domain for liblzma; package contains mixed licenses |

The Phase A build disables the MAVSDK server and curl, so their optional
dependency sets are outside this inventory. MAVSDK's patched MAVLink build uses
the pymavlink generator source nested in the pinned MAVLink checkout instead of
running a build-time `pip install`; generator packages are not linked into the
smoke executable.

The checker `pixi run check-mavsdk-phase-a` fails if reviewed gitlinks,
dependency references, archive hashes/timestamp handling, the pinned-generator
patch, NOTICE component names, or any bundled licence text changes without an
explicit audit update. The complete selected-build texts and their checked hashes
are in `licenses/mavsdk-phase-a/`. MAVSDK-Proto is not fetched, compiled or linked
while the server remains disabled; enabling it requires a new audit.

## Hardening evidence - 2026-09-10

The project MAVSDK fork is pinned at
`9884f109533f564bc6250e5471e6301d3a62f4a7`. The fork replaces the mutable
PicoSHA2 branch with an immutable commit, adds SHA-256 archive checks for
liblzma and nlohmann JSON, handles deterministic archive extraction on both
pre-3.24 and newer CMake, and removes MAVLink's build-time network package
resolution in favor of its pinned nested pymavlink generator source. NOMAD pins
that revision directly so a recursive clean checkout resolves the reviewed graph.

Before publication, a Windows Release configure and smoke-target rebuild completed
against the original three-file dependency pin. The deterministic peer fixture
accepted the expected ArduPilot-like system, rejected a wrong system ID, and failed
on an absent peer. The provenance checker and 12 focused Python tests passed; the
complete C++ suite passed 10/10. Those results predate the subsequent legacy-CMake
and pinned-generator compatibility changes and therefore do not qualify the current
fork revision; fresh hosted evidence is required.

The historical warm build tree was 379,308,757 bytes and the smoke executable was
2,032,128 bytes. These are one-machine diagnostics, not approved budgets or clean-
build benchmarks. Fresh clean-checkout provenance/build evidence, hosted
Linux/Windows, selected ROS image, live ArduPilot SITL and aircraft evidence remain
required.

## Open release blockers

- Re-run provenance, configure/build and deterministic peer qualification from a
  recursive clean checkout at the published NOMAD/MAVSDK pins.
- Re-run the selected dependency and licence audit whenever production parity
  enables another plugin, server, curl or test dependency.
- Release owners must approve build-tree, executable, memory, startup, and CI
  time budgets from repeatable Linux and Windows measurements.
- Hosted Linux/Windows, selected ROS image and live ArduPilot SITL evidence
  remain required.

debt: the selected-build graph is published and pinned; revisit after the
clean-checkout/hosted qualification matrix, and again whenever dependency or
MAVSDK plugin selection changes.
