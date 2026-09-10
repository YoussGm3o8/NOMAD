# MAVSDK Phase A dependency inventory

This inventory records the optional Phase A build inputs at the reviewed NOMAD
gitlink. It is an engineering and notice audit, not a release approval. The
production core does not use MAVSDK yet.

| Component | Reviewed source | License found in fetched source |
|---|---|---|
| MAVSDK | NOMAD gitlink `34b417d45c2c33ce0414bc1bc61b54010d055224` | BSD-3-Clause |
| MAVSDK proto | nested gitlink `1fd0bc7a05c21336227b1eab266b8b610401cf38` | BSD-3-Clause |
| Asio | tag `asio-1-30-2` | Boost-1.0 |
| fmt | tag `12.1.0` | MIT |
| libevents | commit `840a88ea226d4eb0fd4c391ce860317422756435` | BSD-3-Clause |
| libmavlike | commit `90498b14262137ae10b633705810e81bdb85de9c` | BSD-3-Clause |
| MAVLink | commit `d6a7eeaf43319ce6da19a1973ca40180a4210643` | generator (L)GPL-3.0 with MIT output exception |
| nlohmann JSON | archive tag `v3.12.0` | MIT |
| PicoSHA2 | branch `cmake-install-support` | MIT |
| tinyxml2 | tag `11.0.0` | Zlib |
| liblzma from XZ Utils | archive `5.4.5` | public domain for liblzma; package contains mixed licenses |

The Phase A build disables the MAVSDK server and curl, so their optional
dependency sets are outside this inventory. Generated/build-only Python tools
used by the MAVLink dependency are not linked into the smoke executable.

The checker `pixi run check-mavsdk-phase-a` fails if reviewed gitlinks,
dependency references, or NOTICE component names change without an explicit
audit update.

## Open release blockers

- PicoSHA2 follows a branch instead of an immutable commit.
- The nlohmann JSON archive has no checksum in the reviewed CMake input.
- The XZ archive uses MD5 rather than a collision-resistant checksum.
- A redistribution package must carry the applicable complete license texts;
  the root NOTICE is an inventory and does not replace them.
- Release owners must approve build-tree, executable, memory, startup, and CI
  time budgets from repeatable Linux and Windows measurements.

debt: Phase A accepts the reviewed upstream references only as an isolated
experiment; revisit before production cutover or whenever a reference changes;
then pin immutable content, add strong archive hashes, assemble distribution
licenses, and rerun the build/runtime qualification matrix.
