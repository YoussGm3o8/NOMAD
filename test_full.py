"""
NOMAD Full Integration Test
Starts server, runs tests, then shuts down.
"""

import subprocess
import time
import sys
import os
import signal

os.chdir(r'c:\Users\Youssef\Documents\Code\MAD\NOMAD')

print("=" * 60)
print("  NOMAD Full Integration Test")
print("=" * 60)

# Start server as subprocess
print("\n[1] Starting Edge Core server...")
server_env = os.environ.copy()
server_env['NOMAD_SIM_MODE'] = 'true'
server_env['PYTHONPATH'] = os.getcwd()

server_proc = subprocess.Popen(
    [sys.executable, '-m', 'edge_core.main', '--sim', '--no-vision', '--port', '8001'],
    env=server_env,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP  # Windows
)

# Wait for server to start
print("    Waiting for server startup...")
time.sleep(4)

# Check if server is running
if server_proc.poll() is not None:
    print("    ERROR: Server failed to start!")
    output = server_proc.stdout.read() if server_proc.stdout else ""
    print(f"    Output: {output[:500]}")
    sys.exit(1)

print("    Server started on http://127.0.0.1:8001")

# Run tests
print("\n[2] Running API tests...")
tests_passed = 0
tests_failed = 0

try:
    import httpx
    BASE_URL = "http://127.0.0.1:8001"

    # Test: Root
    print("\n    [Test] GET /")
    try:
        r = httpx.get(f"{BASE_URL}/", timeout=5)
        if r.status_code == 200:
            print(f"    [PASS] - {r.json()}")
            tests_passed += 1
        else:
            print(f"    [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"    [ERROR] - {e}")
        tests_failed += 1

    # Test: Health
    print("\n    [Test] GET /health")
    try:
        r = httpx.get(f"{BASE_URL}/health", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"    [PASS] - status={data.get('status')}")
            tests_passed += 1
        else:
            print(f"    [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"    [ERROR] - {e}")
        tests_failed += 1

    # Test: Status
    print("\n    [Test] GET /status")
    try:
        r = httpx.get(f"{BASE_URL}/status", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"    [PASS] - mode={data.get('flight_mode')}, connected={data.get('connected')}")
            tests_passed += 1
        else:
            print(f"    [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"    [ERROR] - {e}")
        tests_failed += 1

    # Test: Landmarks
    print("\n    [Test] GET /api/task/1/landmarks")
    try:
        r = httpx.get(f"{BASE_URL}/api/task/1/landmarks", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"    [PASS] - {data.get('count')} landmarks")
            tests_passed += 1
        else:
            print(f"    [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"    [ERROR] - {e}")
        tests_failed += 1

    # Test: Task 2 Reset
    print("\n    [Test] POST /api/task/2/reset_map")
    try:
        r = httpx.post(f"{BASE_URL}/api/task/2/reset_map", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"    [PASS] - cleared={data.get('cleared_count')}")
            tests_passed += 1
        else:
            print(f"    [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"    [ERROR] - {e}")
        tests_failed += 1

    # Test: Task 2 Target Hit
    print("\n    [Test] POST /api/task/2/target_hit")
    try:
        r = httpx.post(
            f"{BASE_URL}/api/task/2/target_hit",
            json={"x": 1.5, "y": 2.0, "z": 0.3, "label": "test"},
            timeout=5
        )
        if r.status_code == 200:
            data = r.json()
            print(f"    [PASS] - new={data.get('is_new_target')}, total={data.get('total_targets')}")
            tests_passed += 1
        else:
            print(f"    [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"    [ERROR] - {e}")
        tests_failed += 1

finally:
    # Shutdown server
    print("\n[3] Shutting down server...")
    try:
        # Windows: use CTRL_BREAK_EVENT
        server_proc.send_signal(signal.CTRL_BREAK_EVENT)
        server_proc.wait(timeout=5)
    except:
        server_proc.kill()
        server_proc.wait()
    print("    Server stopped.")

# Results
print("\n" + "=" * 60)
print(f"  Results: {tests_passed} passed, {tests_failed} failed")
print("=" * 60)

sys.exit(0 if tests_failed == 0 else 1)
