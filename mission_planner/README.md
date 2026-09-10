# Mission Planner integration

`mission_planner/` is the current C# ground-station integration. It is
transitional while NOMAD moves vehicle behavior into the standalone C++ core.

The plugin may own:

- operator views and configuration;
- telemetry presentation;
- mission and command controls that call the NOMAD client boundary;
- GCS-native link, video, log, and display workflows.

It must not become a second source of vehicle, mission, or safety logic. The C++
core is the product boundary; Mission Planner is replaceable.

## Build

The plugin requires Windows, MSBuild, .NET Framework 4.8, and Mission Planner
reference assemblies:

```powershell
pixi run build-plugin
```

This task can install/overwrite the local plugin; it is a deployment operation.
Use `lint-plugin` and focused `test-plugin-*` tasks for non-deploying checks.
The current core client spawns a local CLI. Direct gimbal, mode/parameter and
fence paths remain; the integrated target needs one active command owner and
explicit handover (G2). A remote core protocol is not implemented yet.
For CONOPS v1.0, the dedicated GCS display must show live aircraft position and
competition area (AE27-OPS-004). Existing EmergencyLand and boundary parameter
writes do not establish compliant independent all-mode termination; their
ownership and acceptance gaps are recorded in migration GAP-05/06. No plugin
termination behavior is changed or flight-qualified by the requirements review.

Pure helper checks are available through the `test-plugin-*` Pixi tasks. See
[the canonical architecture](../docs/architecture.md) and
[development workflow](../docs/development.md) for ownership and verification.
