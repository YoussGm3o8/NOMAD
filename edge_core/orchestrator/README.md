# Orchestrator

Responsibilities:
- FastAPI endpoints: /arm, /disarm, /mode/{flight_mode}, /task/{id}/start, /payload/trigger.
- Watchdog for vision/VIO process; restart without breaking mavlink.
- Hybrid time sync: Chrony (primary), GPS fallback.
- Task 1 and Task 2 coordination to FC.

Keep modules split: API routes, services, adapters, domain models.
