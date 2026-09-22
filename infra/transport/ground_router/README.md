# Ground Multi-Link Router

This Windows/.NET Framework router shares its MP-independent C# implementation
between the Mission Planner plugin and `nomad-link-router.exe`. There are no
Mission Planner assembly dependencies in this directory. The namespace retains
its historical spelling for source compatibility. Product ownership is defined
in [architecture](../../../docs/architecture.md); deployment policy is in
[operations](../../../docs/operations.md).

## Build and run

```powershell
pixi run build-ground-router
build/ground-router/nomad-link-router.exe infra/transport/ground_router/example.json
pixi run test-ground-router
```

Build requires Visual Studio MSBuild/Roslyn and the .NET Framework targeting pack.
The output includes `Nomad.LinkRouter.dll`; keep it beside the executable.
No install, service registration, deployment or aircraft connection is performed
by the build or tests. The smoke test uses only loopback peers.

The host accepts `status`, `select <id>`, `auto`, and `stop` on standard input;
Ctrl+C also stops it. An invalid selection prints `REJECTED`. Closing standard
input does not stop an independently supervised host. SIGKILL/process termination
relies on OS socket cleanup; `stop` exercises orderly worker shutdown.

## Topology and socket ownership

Before this change, plugin-owned `GroundLinkRouter` bound LTE UDP `14560`,
RadioMaster UDP `14550` (or opened a TCP/COM connection), and loopback UDP `14600`.
Mission Planner used UDPCl to `14600` from an ephemeral source port; the router
remembered only the most recent sender. `NomadCoreClient` separately launched
one-shot MAVSDK CLI processes whose default listener was also `14550`, conflicting
with RadioMaster when run on the same computer. The aircraft-side
`infra/transport/mavlink_router` configuration belongs to a different host.

The [example](example.json) has the following ownership on the ground computer:

| Endpoint | Binder | Direction |
|---|---|---|
| `0.0.0.0:14560` | Router, physical `lte` | Aircraft traffic in; selected outbound replies to learned peer |
| `0.0.0.0:14550` | Router, physical `radiomaster` | Same, independently monitored |
| `0.0.0.0:14570` | Router, physical `wifi` | Same, independently monitored |
| `127.0.0.1:14600` | Router, consumer `mission_planner` | MP UDPCl sends here; downlink returns to MP's ephemeral socket |
| Ephemeral MP port | Mission Planner | Receives telemetry and sends native GCS MAVLink |
| `127.0.0.1:14602` | Router, consumer `nomad_core` | Sends downlink to `14601`; accepts outbound only from `14601` |
| `127.0.0.1:14601` | One C++ CLI/runtime process | `udpin:127.0.0.1:14601`; MAVSDK learns router peer `14602` |

Never run the embedded and standalone router with the same configuration
simultaneously. In standalone mode disable the plugin's Multi-Link/embedded-router
toggle and connect native Mission Planner using UDPCl to `14600`. The standalone
router survives Mission Planner exit. **The plugin does not yet remotely display
or control the standalone host's status.** Embedded mode still has plugin-owned
lifetime. Its Link Status cards and manual controls cover the configured collection.

## Configuration

JSON is used by both the existing plugin configuration and the host. `Links`
contains between 1 and 64 entries, at least one enabled. Each link has:

- `Id`: unique, nonempty, case-sensitive stable string; no enum extension required.
- `Name`: optional display name; `Enabled`: default true.
- `Transport`: `UDP`, `TCP`, or `COM`.
- `Port`: UDP listener port or TCP server port; `BindAddress`: IPv4 UDP bind address.
- `RemoteHost`: optional numeric UDP peer address, or required TCP hostname/address.
  A configured UDP peer also requires `RemotePort`; otherwise replies use the
  latest UDP sender. Configure a fixed peer when source restriction is required.
- `Device` and `BaudRate`: serial port configuration for `COM` transport.
- `Priority`: higher is preferred; `ReconnectSeconds`: positive retry interval.

`PreferredLink` optionally ranks one enabled ID ahead of configured priorities.
Set it to `""` when selection should use priorities alone. The compatibility
constants `LTE` and `RadioMaster` are strings, not the router's identity model.
Omitting `Links` translates legacy `LteBindPort`, `RadioBindPort`, COM/TCP settings
into those two IDs. An explicitly empty/invalid collection is rejected.

`Consumers` contains 1–32 entries with unique `Id`, `RouterPort`, and optional
`ClientPort`. Every router consumer socket binds IPv4 loopback. `ClientPort = 0`
learns one loopback peer; another peer can replace it only after three seconds
without traffic from the old peer, resetting parser state. A fixed client port
accepts only that endpoint and receives downlink without first sending anything.
Each consumer owns its own parser. Local commands are never copied to other local
consumers. Exact downlink reflections within the 750 ms dedup window are dropped.
Do not add an external relay that loops traffic back after that window.

Configuration rejects overlapping physical UDP/router/client ports and physical
UDP destinations that point into the local topology. Port binds are exclusive.
A failed physical connection retries independently; a local bind failure aborts
startup and closes resources already opened. Missing consumers do not stop delivery.
Structural configuration changes require restart; per-link settings are copied
at construction. Local loopback access is a trust boundary, not authentication.

In the plugin JSON, use `RouterLinks` and `RouterConsumers` for these collections,
and `PreferredMavlinkLink` for the preferred ID. The legacy settings UI remains
available for two-link configurations; edit/import JSON to add arbitrary links.
`RouterLocalPort` continues to set the `mission_planner` consumer's router port.
Legacy default core endpoint `udpin:0.0.0.0:14550` migrates to `14601` when the
embedded router is enabled; explicit other endpoints remain unchanged. Review
custom endpoint/consumer pairs together. One-shot CLI processes must not run
concurrently on the same listener.

## Selection, delivery, and transactions

All enabled links are read by one owned worker with bounded polling batches.
TCP connect attempts are asynchronous; per-link reopen state and parser resets
are independent. Complete frames establish freshness, not arbitrary incoming
bytes. UDP silence expires after `HeartbeatTimeoutSec` (default 3 seconds).
Sequence gaps are tracked per physical link and `(sysid, compid)`; duplicates
and backward/out-of-order sequence deltas are ignored. `LatencyMs` is retained
for compatibility and measures heartbeat interval deviation, **not RTT**.

Automatic selection ranks usable, non-critical links ahead of critical ones,
then preferred ID, priority, and ordinal ID. A stale active link is replaced
immediately by the best receiving link, regardless of cooldown. Healthy-link
recovery requires `PreferredLinkReconnectDelaySec` (default 10) continuously
healthy plus `FailoverCooldownSec` (default 2). `AutoReconnectPreferred = false`
retains a usable active link. Manual selection pins an enabled ID; unknown or
disabled IDs fail without changing state. A dead manual selection drops outbound
rather than silently using another link. `auto` releases the pin.

Only active-source telemetry is distributed, apart from pinned parameter replies.
A bounded cache records frames actually delivered, including their complete v1/v2
bytes, identity, sequence, payload, checksum and signature. Cross-link copies are
suppressed for 750 ms across handovers; standby-first copies cannot suppress an
active delivery. Same-link repeats retain upstream semantics. At 16,384 cache
entries new telemetry is dropped until entries expire. The framing parser does
not validate CRC/signatures; endpoint software remains responsible for validation.

debt: bounded polling and 16,384 recent frames; revisit when measured traffic
exceeds 20,000 frames/second or link count exceeds 64; then qualify queued I/O
and explicit per-consumer backpressure against the same no-fan-out tests.

**Outbound MAVLink is sent through one selected physical link and is not broadcast
across all healthy links.** A failed send is not retried on another transport:
partial TCP/serial delivery may already have happened. The caller decides retries.
Before the first telemetry frame, an open TCP/COM or configured UDP destination
can carry initial GCS announcements on one selected link. A previously receiving
but now stale link does not use this startup exception.

`PARAM_*` and `PARAM_EXT_*` requests/writes pin one physical link globally across
consumers. Parameter traffic stays pinned across automatic/manual changes.
Replies and successful outbound traffic refresh a four-second inactivity lease.
If the pinned link becomes stale, outgoing parameter frames are dropped and do
not extend the lease; after expiry a new request may select the current link.
Unsolicited parameter replies after expiry follow the current active source.
This is not request correlation or concurrent-client arbitration. Mission/fence
and FTP traffic use the active link per frame; no transaction pinning is claimed
for them. Their callers must detect failure and restart/verify exchanges.

## Limits and next step

The router owns transport selection, not command admission, payload policy,
aircraft qualification or mission sequencing. MP native controls, pilot/RC and
ArduPilot remain external authorities. C++ owns NOMAD validation, safety, deadlines
and verified outcomes. There is no global single-writer authority or persistent
C++ IPC here; integrated operation still requires handover and inhibition.
Shared hardware/power/network paths do not provide independent redundancy.

Next: add a versioned local status/config control client for the standalone host,
then integrate the persistent C++ runtime's typed requests and explicit authority
handover as a separately reviewed change. QuadPlane transition qualification
remains the active aircraft item and is untouched by this transport extraction.
