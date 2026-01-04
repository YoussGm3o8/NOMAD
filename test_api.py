"""
Quick API test for NOMAD.
Starts server, runs tests, and shuts down.
"""

import asyncio
import sys
import time
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

def test_api():
    """Test the API endpoints."""
    import httpx
    
    base_url = "http://127.0.0.1:8766"
    results = []
    
    print("\n" + "=" * 60)
    print("Testing API Endpoints...")
    print("=" * 60)
    
    # Wait for server to start
    time.sleep(1)
    
    try:
        # Test 1: Health endpoint
        print("\nTest 1: GET /health")
        response = httpx.get(f"{base_url}/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"  [PASS] Status: {data}")
            results.append(True)
        else:
            print(f"  [FAIL] Status code: {response.status_code}")
            results.append(False)
        
        # Test 2: Status endpoint
        print("\nTest 2: GET /status")
        response = httpx.get(f"{base_url}/status", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"  [PASS] Flight mode: {data.get('flight_mode')}, Connected: {data.get('connected')}")
            results.append(True)
        else:
            print(f"  [FAIL] Status code: {response.status_code}")
            results.append(False)
        
        # Test 3: Task 2 hit endpoint (POST)
        print("\nTest 3: POST /api/task/2/target_hit")
        response = httpx.post(
            f"{base_url}/api/task/2/target_hit",
            json={"x": 1.0, "y": 2.0, "z": 0.5, "label": "test_target"},
            timeout=5
        )
        if response.status_code == 200:
            data = response.json()
            print(f"  [PASS] Success: {data.get('success')}, New: {data.get('is_new_target')}")
            results.append(True)
        else:
            print(f"  [FAIL] Status code: {response.status_code}")
            results.append(False)
        
        # Test 4: Task 2 check same target (should not be new)
        print("\nTest 4: POST /api/task/2/target_hit (same location)")
        response = httpx.post(
            f"{base_url}/api/task/2/target_hit",
            json={"x": 1.0, "y": 2.0, "z": 0.5},
            timeout=5
        )
        if response.status_code == 200:
            data = response.json()
            is_not_new = not data.get('is_new_target')
            if is_not_new:
                print(f"  [PASS] Duplicate target detected (is_new_target=False)")
                results.append(True)
            else:
                print(f"  [FAIL] Should have detected duplicate")
                results.append(False)
        else:
            print(f"  [FAIL] Status code: {response.status_code}")
            results.append(False)
        
        # Test 5: Task 2 exclusion map
        print("\nTest 5: GET /api/task/2/exclusion_map")
        response = httpx.get(f"{base_url}/api/task/2/exclusion_map", timeout=5)
        if response.status_code == 200:
            data = response.json()
            total = data.get('total_targets', 0)
            print(f"  [PASS] Exclusion map has {total} targets")
            results.append(True)
        else:
            print(f"  [FAIL] Status code: {response.status_code}")
            results.append(False)
            
    except Exception as e:
        print(f"  [ERROR] {e}")
        results.append(False)
    
    return all(results), len(results), sum(results)


def run_server():
    """Run the uvicorn server."""
    import uvicorn
    from edge_core.api import create_app
    from edge_core.state import StateManager
    
    sm = StateManager()
    app = create_app(sm)
    uvicorn.run(app, host='127.0.0.1', port=8766, log_level='warning')


def main():
    print("=" * 60)
    print("  NOMAD API Integration Test")
    print("=" * 60)
    
    # Check if httpx is available
    try:
        import httpx
    except ImportError:
        print("\n[SKIP] httpx not installed. Install with: pip install httpx")
        print("Running import-only API test...")
        
        from edge_core.api import create_app
        from edge_core.state import StateManager
        
        sm = StateManager()
        app = create_app(sm)
        print(f"\n[PASS] API app created: {app.title}")
        print("\nTo fully test the API, install httpx and run again.")
        return True
    
    # Start server in background thread
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    
    # Run tests
    try:
        all_passed, total, passed = test_api()
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        return False
    
    # Summary
    print("\n" + "=" * 60)
    print("  API TEST SUMMARY")
    print("=" * 60)
    print(f"  Passed: {passed}/{total}")
    print(f"  Status: {'PASS' if all_passed else 'FAIL'}")
    print("=" * 60)
    
    return all_passed


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
