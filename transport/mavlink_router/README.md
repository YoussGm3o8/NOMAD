# mavlink-router

- Purpose: split FC UART (921600) to local UDP endpoints and ground VPN.
- Outputs: 127.0.0.1:14550 (Orchestrator), 127.0.0.1:14551 (Vision/VIO), <VPN_IP>:14550 (Mission Planner).
- Add `mavlink-router.conf` and service files here.
