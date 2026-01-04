"""
Simple local test for NOMAD Edge Core API.
Run this to verify the API is working.
"""

import httpx
import sys

BASE_URL = "http://127.0.0.1:8000"

def main():
    print("\n" + "=" * 60)
    print("  NOMAD Edge Core API Test")
    print("=" * 60)
    print(f"\nTesting API at: {BASE_URL}\n")

    tests_passed = 0
    tests_failed = 0

    # Test 1: Root endpoint
    print("[Test 1] GET / (Root)")
    try:
        r = httpx.get(f"{BASE_URL}/", timeout=5)
        if r.status_code == 200:
            print(f"  [PASS] - {r.json()}")
            tests_passed += 1
        else:
            print(f"  ✗ FAIL - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"  ✗ ERROR - {e}")
        tests_failed += 1

    # Test 2: Health check
    print("\n[Test 2] GET /health")
    try:
        r = httpx.get(f"{BASE_URL}/health", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"  [PASS] - status={data.get('status')}, connected={data.get('connected')}")
            tests_passed += 1
        else:
            print(f"  [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"  [ERROR] - {e}")
        tests_failed += 1

    # Test 3: Status
    print("\n[Test 3] GET /status")
    try:
        r = httpx.get(f"{BASE_URL}/status", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"  [PASS] - flight_mode={data.get('flight_mode')}, battery={data.get('battery_voltage')}V")
            tests_passed += 1
        else:
            print(f"  [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"  [ERROR] - {e}")
        tests_failed += 1

    # Test 4: Landmarks
    print("\n[Test 4] GET /api/task/1/landmarks")
    try:
        r = httpx.get(f"{BASE_URL}/api/task/1/landmarks", timeout=5)
        if r.status_code == 200:
            data = r.json()
            count = data.get('count', 0)
            print(f"  [PASS] - {count} landmarks loaded")
            for lm in data.get('landmarks', [])[:3]:
                print(f"    - {lm['name']}")
            tests_passed += 1
        else:
            print(f"  [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"  [ERROR] - {e}")
        tests_failed += 1

    # Test 5: Task 2 - Reset Map
    print("\n[Test 5] POST /api/task/2/reset_map")
    try:
        r = httpx.post(f"{BASE_URL}/api/task/2/reset_map", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"  [PASS] - cleared={data.get('cleared_count')}")
            tests_passed += 1
        else:
            print(f"  [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"  [ERROR] - {e}")
        tests_failed += 1

    # Test 6: Task 2 - Register target hit
    print("\n[Test 6] POST /api/task/2/target_hit")
    try:
        r = httpx.post(
            f"{BASE_URL}/api/task/2/target_hit",
            json={"x": 1.5, "y": 2.0, "z": 0.3, "label": "test_target"},
            timeout=5
        )
        if r.status_code == 200:
            data = r.json()
            print(f"  [PASS] - new={data.get('is_new_target')}, total={data.get('total_targets')}")
            tests_passed += 1
        else:
            print(f"  [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"  [ERROR] - {e}")
        tests_failed += 1

    # Test 7: Task 2 - Exclusion map
    print("\n[Test 7] GET /api/task/2/exclusion_map")
    try:
        r = httpx.get(f"{BASE_URL}/api/task/2/exclusion_map", timeout=5)
        if r.status_code == 200:
            data = r.json()
            print(f"  [PASS] - {data.get('total_targets')} targets in map")
            tests_passed += 1
        else:
            print(f"  [FAIL] - Status {r.status_code}")
            tests_failed += 1
    except Exception as e:
        print(f"  [ERROR] - {e}")
        tests_failed += 1

    # Summary
    print("\n" + "=" * 60)
    print(f"  Results: {tests_passed} passed, {tests_failed} failed")
    print("=" * 60)

    return 0 if tests_failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
