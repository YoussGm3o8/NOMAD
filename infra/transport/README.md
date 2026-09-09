# Transport configuration

mavlink_router contains deployment routing configuration. Routers may bridge
serial/radio/TCP legs into UDP without owning vehicle decisions.

The current C++ connection is UDP-only. MAVSDK adoption adds qualified transports
without changing Vehicle policy; native serial/TCP support is not yet production
behavior. Validate loop prevention, duplicate handling and link capacity at G3.
Keep real endpoints in ignored configuration. See
[operations](../../docs/operations.md) and [architecture](../../docs/architecture.md).
