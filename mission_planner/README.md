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
`NomadCoreClient` supports `LegacyOneShot` and `PersistentRuntime`. Persistent
mode connects to the C++ runtime over versioned loopback JSON Lines IPC, performs
HELLO negotiation, then issues typed requests without spawning `nomad`. It does
not automatically retry a request whose response is lost. The compatibility
mode remains available until a deployment selects persistent mode. The runtime
protocol is local-only and does not authenticate clients. Direct gimbal,
mode/parameter and fence paths remain; global authority handover is still open.
For CONOPS v1.0, the dedicated GCS display must show live aircraft position and
competition area (AE27-OPS-004). Existing EmergencyLand and boundary parameter
writes do not establish compliant independent all-mode termination; their
ownership and acceptance gaps are recorded in migration GAP-05/06. No plugin
termination behavior is changed or flight-qualified by the requirements review.

Core-client loopback protocol checks are available through
`pixi run test-plugin-core-client`; the other pure helper checks are available
through `test-plugin-*` Pixi tasks. See
[the canonical architecture](../docs/architecture.md) and
[development workflow](../docs/development.md) for ownership and verification.

## Multi-Link routing configuration

Link Status now renders configured physical links and provides manual selection
per stable ID. Existing LTE/RadioMaster fields remain a compatibility input; add
`RouterLinks` and `RouterConsumers` to the plugin JSON for additional links.
See the [shared router reference](../infra/transport/ground_router/README.md) for
field names, a complete host JSON example, port ownership and recovery policy.
The plugin's default C++ listener is now loopback `14601`, separate from physical
RadioMaster `14550`; the router feeds it from `14602`.

For standalone ownership set `RouterMode` to `Standalone`, configure the
loopback management endpoint (default `127.0.0.1:14610`), and connect native MP
via UDPCl to the host's `14600`. MP restart then leaves the host running while
the plugin reconnects to status/events and marks stale data explicitly. The UI
can select an enabled link or return to automatic selection; endpoint, consumer,
and policy changes require a host restart. Embedded mode remains the default and
keeps plugin-owned lifetime. Do not start both modes against the same endpoints
or simultaneous CLI processes that bind the same consumer endpoint.
