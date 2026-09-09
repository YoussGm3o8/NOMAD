# Security policy

## Current trust boundary

The working tree removes Edge Core and its remote API. The C++ CLI only checks
that NOMAD_API_KEY is nonempty; this is an operator opt-in, not verification of
identity or a remote credential. Local OS permissions currently protect executable
and configuration access. Core library calls do not inherit CLI authentication
or audit.

Authenticated/authorized local and remote clients, replay protection, complete
outcome audit and exposed ROS/media endpoint protection remain release gates.
A VPN is an optional network control; it does not authorize commands by itself.
The retained video tool's HTTP controls currently lack authentication.

See [operations](docs/operations.md), [architecture](docs/architecture.md) and
[the safety case](docs/safety.md) for canonical controls and open evidence.
Do not deploy template development credentials or infer security from a config
flag. Independent manual control must be qualified for the selected aircraft;
this plan prescribes no airborne motor-kill action.

## Reporting

Report command injection, authentication bypass, credential leakage, unsafe
replay or remote execution privately to repository maintainers using their
configured private security reporting channel. Avoid public disclosure of
exploitable details, credentials, aircraft locations or private datasets.
Do not invent an issue type or publish a vulnerability if a private channel
is unavailable.

## Operator responsibilities

Restrict command, MAVLink, DDS, media and maintenance endpoints to intended
principals. Protect/rotate actual secrets outside source control, validate
server identity and keep dependency/firmware pairs pinned and qualified.
Preserve sanitized evidence of security failures without recording secrets.
Current development support does not imply a flight-qualified release.
