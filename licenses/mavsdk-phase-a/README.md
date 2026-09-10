# MAVSDK Phase A redistribution licenses

This directory accompanies any NOMAD distribution that contains the optional
statically linked MAVSDK Phase A executable. The files are verbatim copies from
the exact fetched sources recorded in
[the dependency inventory](../../docs/mavsdk-dependencies.md). The provenance
checker validates every file by SHA-256 so a missing or changed text fails the
Phase A gate.

| File | Component and licence |
|---|---|
| `MAVSDK-BSD-3-Clause.txt` | MAVSDK, BSD-3-Clause |
| `Asio-Boost-1.0.txt` | Asio, Boost Software License 1.0 |
| `fmt-MIT.txt` | fmt, MIT |
| `libevents-BSD-3-Clause.txt` | libevents, BSD-3-Clause |
| `libmavlike-BSD-3-Clause.txt` | libmavlike, BSD-3-Clause |
| `MAVLink-generator-and-output.txt` | MAVLink generator (L)GPLv3 and generated-output MIT exception |
| `nlohmann-json-MIT.txt` | nlohmann JSON, MIT |
| `PicoSHA2-MIT.txt` | PicoSHA2, MIT |
| `tinyxml2-Zlib.txt` | tinyxml2, Zlib |
| `XZ-COPYING.txt` | XZ Utils licensing summary; liblzma is public domain |
| `XZ-GPL-2.0.txt` | XZ Utils source-package GPLv2 text |
| `XZ-GPL-3.0.txt` | XZ Utils source-package GPLv3 text |
| `XZ-LGPL-2.1.txt` | XZ Utils source-package LGPLv2.1 text |

The MAVSDK server is disabled for Phase A, so MAVSDK-Proto is not fetched,
compiled or linked into this executable. Re-audit this bundle before enabling
the server, curl support, tests, another plugin, or another dependency set.
