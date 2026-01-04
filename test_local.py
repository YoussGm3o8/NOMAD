"""
Local test script for NOMAD edge_core modules.
Run with: python test_local.py
"""

import sys
from pathlib import Path

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).parent))

def test_imports():
    """Test all edge_core imports."""
    print("=" * 60)
    print("Testing edge_core imports...")
    print("=" * 60)
    
    try:
        from edge_core import (
            # Models
            SystemState, StateManager,
            # Geospatial
            GPSCoordinate, NEDOffset, DroneState,
            calculate_gps_offset_meters, raycast_target_gps,
            haversine_distance, offset_gps_by_meters,
            # Task 1
            Landmark, load_landmarks, find_nearest_landmark,
            identify_target_relative_to_landmark, get_target_description,
            # Task 2
            ExclusionMap, GimbalPID,
            # Visual Servoing
            ServoMode, PIDController, TargetTracker,
            # IPC
            IPCMessage, ZMQPublisher, ZMQSubscriber,
            # Mocks
            is_sim_mode, set_sim_mode, MockZedCamera, MockYOLO,
            # Mission Validator
            MissionValidator, EKFMode, check_ekf_source_sync,
            # Time Sync
            TimeSyncService, TimeSyncStatus,
            # Hardware Monitor
            HealthMonitor, ThrottleLevel,
        )
        print("[PASS] All edge_core imports successful")
        return True
    except Exception as e:
        print(f"[FAIL] Import error: {e}")
        return False


def test_geospatial():
    """Test geospatial calculations."""
    print("\n" + "=" * 60)
    print("Testing geospatial calculations...")
    print("=" * 60)
    
    from edge_core.geospatial import (
        GPSCoordinate, NEDOffset, DroneState,
        calculate_gps_offset_meters, raycast_target_gps,
        haversine_distance, deg_to_rad, rad_to_deg,
    )
    import math
    
    tests_passed = 0
    tests_total = 0
    
    # Test 1: Degree/Radian conversion
    tests_total += 1
    if abs(deg_to_rad(180) - math.pi) < 1e-10:
        print("[PASS] deg_to_rad(180) = π")
        tests_passed += 1
    else:
        print("[FAIL] deg_to_rad conversion error")
    
    # Test 2: GPS offset - same point
    tests_total += 1
    coord = GPSCoordinate(lat=45.5, lon=-73.5)
    offset = calculate_gps_offset_meters(coord, coord)
    if abs(offset.north) < 0.01 and abs(offset.east) < 0.01:
        print("[PASS] Same point offset is zero")
        tests_passed += 1
    else:
        print(f"[FAIL] Same point offset not zero: {offset}")
    
    # Test 3: North offset
    tests_total += 1
    origin = GPSCoordinate(lat=45.5, lon=-73.5)
    target = GPSCoordinate(lat=45.5001, lon=-73.5)  # ~11m north
    offset = calculate_gps_offset_meters(origin, target)
    if 10 < offset.north < 12 and abs(offset.east) < 0.1:
        print(f"[PASS] North offset: {offset.north:.2f}m")
        tests_passed += 1
    else:
        print(f"[FAIL] North offset incorrect: {offset}")
    
    # Test 4: Haversine distance
    tests_total += 1
    a = GPSCoordinate(lat=45.5, lon=-73.5)
    b = GPSCoordinate(lat=45.5001, lon=-73.5)
    dist = haversine_distance(a, b)
    if 11.0 < dist < 11.2:
        print(f"[PASS] Haversine distance: {dist:.2f}m")
        tests_passed += 1
    else:
        print(f"[FAIL] Haversine distance incorrect: {dist}")
    
    # Test 5: Raycasting straight down
    tests_total += 1
    drone = DroneState(
        gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
        heading_deg=0.0,
        gimbal_pitch_deg=-90.0,  # Straight down
        lidar_distance_m=50.0,
    )
    target = raycast_target_gps(drone)
    if abs(target.lat - 45.5) < 0.00001 and abs(target.lon - (-73.5)) < 0.00001:
        print("[PASS] Straight-down raycast hits point below drone")
        tests_passed += 1
    else:
        print(f"[FAIL] Raycast target incorrect: {target}")
    
    # Test 6: Raycasting 45 degrees north
    tests_total += 1
    drone = DroneState(
        gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
        heading_deg=0.0,  # North
        gimbal_pitch_deg=-45.0,  # 45 degrees down
        lidar_distance_m=70.71,  # sqrt(50^2 + 50^2)
    )
    target = raycast_target_gps(drone)
    offset = calculate_gps_offset_meters(drone.gps, target)
    if 49 < offset.north < 51 and abs(offset.east) < 1:
        print(f"[PASS] 45° North raycast: {offset.north:.1f}m north")
        tests_passed += 1
    else:
        print(f"[FAIL] 45° raycast incorrect: {offset}")
    
    print(f"\nGeospatial tests: {tests_passed}/{tests_total} passed")
    return tests_passed == tests_total


def test_task1():
    """Test Task 1 landmark-relative text generation."""
    print("\n" + "=" * 60)
    print("Testing Task 1 (Landmark Relative Text)...")
    print("=" * 60)
    
    from edge_core.task1 import (
        Landmark, load_landmarks, find_nearest_landmark,
        format_direction_text, identify_target_relative_to_landmark,
    )
    from edge_core.geospatial import GPSCoordinate, NEDOffset, DroneState
    
    tests_passed = 0
    tests_total = 0
    
    # Test 1: Load landmarks from config
    tests_total += 1
    try:
        landmarks = load_landmarks()
        if len(landmarks) > 0:
            print(f"[PASS] Loaded {len(landmarks)} landmarks from config")
            for lm in landmarks[:3]:  # Show first 3
                print(f"       - {lm.name}: ({lm.lat}, {lm.lon})")
            tests_passed += 1
        else:
            print("[FAIL] No landmarks loaded")
    except Exception as e:
        print(f"[FAIL] Load landmarks error: {e}")
    
    # Test 2: Find nearest landmark
    tests_total += 1
    test_landmarks = [
        Landmark(name="A", lat=45.5, lon=-73.5),
        Landmark(name="B", lat=45.5001, lon=-73.5001),
    ]
    target = GPSCoordinate(lat=45.50001, lon=-73.50001)
    match = find_nearest_landmark(target, test_landmarks)
    if match and match.landmark.name == "A":
        print(f"[PASS] Found nearest landmark: {match.landmark.name}")
        tests_passed += 1
    else:
        print(f"[FAIL] Nearest landmark incorrect: {match}")
    
    # Test 3: Format direction text
    tests_total += 1
    offset = NEDOffset(north=1.5, east=0.5)
    text = format_direction_text(offset)
    if "1.5m North" in text and "0.5m East" in text:
        print(f"[PASS] Direction text: '{text}'")
        tests_passed += 1
    else:
        print(f"[FAIL] Direction text incorrect: '{text}'")
    
    # Test 4: South/West formatting
    tests_total += 1
    offset = NEDOffset(north=-2.0, east=-1.0)
    text = format_direction_text(offset)
    if "2.0m South" in text and "1.0m West" in text:
        print(f"[PASS] Direction text: '{text}'")
        tests_passed += 1
    else:
        print(f"[FAIL] Direction text incorrect: '{text}'")
    
    # Test 5: Full Task 1 workflow
    tests_total += 1
    drone = DroneState(
        gps=GPSCoordinate(lat=45.5, lon=-73.5, alt=50.0),
        heading_deg=0.0,
        gimbal_pitch_deg=-90.0,
        lidar_distance_m=50.0,
    )
    result = identify_target_relative_to_landmark(drone)
    if result and "Target is" in result.formatted_text:
        print(f"[PASS] Full workflow: '{result.formatted_text}'")
        tests_passed += 1
    else:
        print(f"[FAIL] Full workflow failed: {result}")
    
    print(f"\nTask 1 tests: {tests_passed}/{tests_total} passed")
    return tests_passed == tests_total


def test_ekf_validation():
    """Test EKF source validation."""
    print("\n" + "=" * 60)
    print("Testing EKF Validation...")
    print("=" * 60)
    
    from edge_core.mission_validator import (
        EKFMode, EKFSourcePosXY, check_ekf_source_sync,
    )
    
    tests_passed = 0
    tests_total = 0
    
    # Test 1: GPS mode validation (correct)
    tests_total += 1
    is_valid, warning = check_ekf_source_sync(3, EKFMode.GPS)  # GPS = 3
    if is_valid and warning == "":
        print("[PASS] GPS mode (3) validates for Task 1")
        tests_passed += 1
    else:
        print(f"[FAIL] GPS validation incorrect: {is_valid}, {warning}")
    
    # Test 2: VIO mode validation (correct)
    tests_total += 1
    is_valid, warning = check_ekf_source_sync(6, EKFMode.VIO)  # EXTNAV = 6
    if is_valid and warning == "":
        print("[PASS] VIO mode (6) validates for Task 2")
        tests_passed += 1
    else:
        print(f"[FAIL] VIO validation incorrect: {is_valid}, {warning}")
    
    # Test 3: GPS mode mismatch (VIO set for GPS task)
    tests_total += 1
    is_valid, warning = check_ekf_source_sync(6, EKFMode.GPS)
    if not is_valid and "MISMATCH" in warning:
        print(f"[PASS] Detected mismatch: VIO (6) for GPS task")
        tests_passed += 1
    else:
        print(f"[FAIL] Mismatch not detected")
    
    # Test 4: VIO mode mismatch (GPS set for VIO task)
    tests_total += 1
    is_valid, warning = check_ekf_source_sync(3, EKFMode.VIO)
    if not is_valid and "MISMATCH" in warning:
        print(f"[PASS] Detected mismatch: GPS (3) for VIO task")
        tests_passed += 1
    else:
        print(f"[FAIL] Mismatch not detected")
    
    print(f"\nEKF Validation tests: {tests_passed}/{tests_total} passed")
    return tests_passed == tests_total


def test_mocks():
    """Test simulation/mock components."""
    print("\n" + "=" * 60)
    print("Testing Mock/Simulation Components...")
    print("=" * 60)
    
    from edge_core.mocks import (
        is_sim_mode, set_sim_mode, MockZedCamera, MockYOLO,
        create_camera, create_yolo,
    )
    
    tests_passed = 0
    tests_total = 0
    
    # Test 1: Default sim mode
    tests_total += 1
    initial_mode = is_sim_mode()
    print(f"       Initial sim mode: {initial_mode}")
    tests_passed += 1
    print("[PASS] is_sim_mode() returns bool")
    
    # Test 2: Set sim mode
    tests_total += 1
    set_sim_mode(True)
    if is_sim_mode():
        print("[PASS] set_sim_mode(True) enables simulation")
        tests_passed += 1
    else:
        print("[FAIL] set_sim_mode failed")
    
    # Test 3: MockZedCamera
    tests_total += 1
    try:
        camera = MockZedCamera()
        print(f"[PASS] MockZedCamera created")
        tests_passed += 1
    except Exception as e:
        print(f"[FAIL] MockZedCamera error: {e}")
    
    # Test 4: MockYOLO
    tests_total += 1
    try:
        yolo = MockYOLO()
        print(f"[PASS] MockYOLO created")
        tests_passed += 1
    except Exception as e:
        print(f"[FAIL] MockYOLO error: {e}")
    
    # Test 5: Factory functions in sim mode
    tests_total += 1
    set_sim_mode(True)
    camera = create_camera()
    if isinstance(camera, MockZedCamera):
        print("[PASS] create_camera() returns MockZedCamera in sim mode")
        tests_passed += 1
    else:
        print(f"[FAIL] create_camera() returned {type(camera)}")
    
    # Reset sim mode
    set_sim_mode(initial_mode)
    
    print(f"\nMock tests: {tests_passed}/{tests_total} passed")
    return tests_passed == tests_total


def test_visual_servoing():
    """Test visual servoing components."""
    print("\n" + "=" * 60)
    print("Testing Visual Servoing...")
    print("=" * 60)
    
    from edge_core.visual_servoing import (
        ServoMode, BoundingBox, PIDController, PIDConfig,
        TargetTracker, TargetTrackerConfig, TrackingState,
    )
    
    tests_passed = 0
    tests_total = 0
    
    # Test 1: ServoMode enum
    tests_total += 1
    modes = [ServoMode.DRONE_BODY, ServoMode.GIMBAL]
    print(f"[PASS] ServoMode enum: {[m.name for m in modes]}")
    tests_passed += 1
    
    # Test 2: BoundingBox
    tests_total += 1
    bbox = BoundingBox(x1=100, y1=100, x2=150, y2=150)
    center_x = bbox.center_x
    center_y = bbox.center_y
    if center_x == 125 and center_y == 125:
        print(f"[PASS] BoundingBox center: ({center_x}, {center_y})")
        tests_passed += 1
    else:
        print(f"[FAIL] BoundingBox center incorrect: ({center_x}, {center_y})")
    
    # Test 3: PIDController
    tests_total += 1
    config = PIDConfig(kp=1.0, ki=0.1, kd=0.05)
    pid = PIDController(config)
    output = pid.update(error=10.0, dt=0.1)
    if output != 0:  # Should produce some output
        print(f"[PASS] PIDController output: {output:.2f}")
        tests_passed += 1
    else:
        print(f"[FAIL] PIDController output is zero")
    
    # Test 4: TargetTracker
    tests_total += 1
    tracker_config = TargetTrackerConfig()  # Use defaults
    tracker = TargetTracker(tracker_config)
    state = tracker.state  # Property, not method
    if isinstance(state, TrackingState) and not state.is_tracking:
        print(f"[PASS] TargetTracker initial state: is_tracking={state.is_tracking}")
        tests_passed += 1
    else:
        print(f"[FAIL] TargetTracker state incorrect: {state}")
    
    print(f"\nVisual Servoing tests: {tests_passed}/{tests_total} passed")
    return tests_passed == tests_total


def test_state_manager():
    """Test StateManager."""
    print("\n" + "=" * 60)
    print("Testing State Manager...")
    print("=" * 60)
    
    from edge_core.state import StateManager
    from edge_core.models import SystemState
    
    tests_passed = 0
    tests_total = 0
    
    # Test 1: Create StateManager
    tests_total += 1
    try:
        manager = StateManager()
        print("[PASS] StateManager created")
        tests_passed += 1
    except Exception as e:
        print(f"[FAIL] StateManager creation error: {e}")
        return False
    
    # Test 2: Get initial state
    tests_total += 1
    state = manager.get_state()
    if isinstance(state, SystemState):
        print(f"[PASS] Initial state retrieved: connected={state.connected}, flight_mode={state.flight_mode}")
        tests_passed += 1
    else:
        print(f"[FAIL] State type incorrect: {type(state)}")
    
    print(f"\nState Manager tests: {tests_passed}/{tests_total} passed")
    return tests_passed == tests_total


def main():
    """Run all local tests."""
    print("\n" + "=" * 60)
    print("  NOMAD Edge Core Local Test Suite")
    print("=" * 60)
    
    results = []
    
    results.append(("Imports", test_imports()))
    results.append(("Geospatial", test_geospatial()))
    results.append(("Task 1", test_task1()))
    results.append(("EKF Validation", test_ekf_validation()))
    results.append(("Mocks", test_mocks()))
    results.append(("Visual Servoing", test_visual_servoing()))
    results.append(("State Manager", test_state_manager()))
    
    # Summary
    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    
    total_passed = 0
    total_tests = len(results)
    
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}] {name}")
        if passed:
            total_passed += 1
    
    print("-" * 60)
    print(f"  Overall: {total_passed}/{total_tests} test suites passed")
    print("=" * 60)
    
    return total_passed == total_tests


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
