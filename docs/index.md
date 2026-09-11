# NOMAD documentation

One C++ core, independent clients, optional onboard or ground compute. The working
tree removes Python Edge Core but has not completed deployment or competition
qualification.

| Subject owner | Document |
|---|---|
| Requirements, provenance and user decisions | [PRD](prd.md) |
| PRD source appendix: official rules, scoring and interpretations | [CONOPS inventory](conops-requirements.md) |
| Target ownership and data flow | [Architecture](architecture.md) |
| AEAC 2027 external server contract and competition module | [AEAC 2027 integration](aeac-2027.md) |
| Source inventory, contradictions and objective gates | [Migration](migration.md) |
| Build, tests and contribution workflow | [Development](development.md) |
| Profiles, runtime procedures and evidence handling | [Operations](operations.md) |
| Hazards and stable requirement/test mappings | [Safety](safety.md) |
| Subordinate transport decision and parity gates | [MAVSDK adoption](mavsdk-adoption.md) |
| Agent practices | [Agent guidance](agent-guidance.md) |

Root PLAN.md is the delivery summary; TODO.md is the only actionable ledger.
Component READMEs describe local interfaces. Historical CHANGELOG entries are
not current qualification evidence. Start with PRD, then architecture and migration.
