# Task 2 Object Detection Implementation Plan (ROS Only)

## 1. Executive Summary
Enable YOLOv8-based object detection for Task 2 (Indoor Extinguish) by activating the existing `IsaacROSBridge`. As per the "ROS Only" constraint, we will rely exclusively on the NVIDIA Isaac ROS pipeline (ZED2i -> Isaac VSLAM -> Isaac YOLO) and will not implement a standalone Python fallback.

## 2. Current State Analysis
*   **Infrastructure**: `edge_core/isaac_ros_bridge.py` is fully implemented but currently dormant.
*   **Integration**: `edge_core/main.py` does not initialize the bridge.
*   **Dependencies**: The system assumes a running ROS2 environment with `isaac_ros_yolov8` and `isaac_ros_visual_slam`.

## 3. Implementation Plan

### Phase 1: Main Application Integration
**Goal**: Initialize the ROS bridge when the Edge Core service starts.
*   **Action**: Update `edge_core/main.py`.
*   **Details**:
    *   Import `init_isaac_bridge` and `set_isaac_bridge`.
    *   Initialize the bridge singleton early in the startup sequence.
    *   Pass the instance to the API module via `set_isaac_bridge`.
    *   Ensure the bridge `start()` method is called to begin the ROS spin loop.
    *   Ensure `stop()` is called in the cleanup handler.

### Phase 2: Configuration & Robustness
**Goal**: Ensure the bridge connects to the correct topics.
*   **Action**: Verify/Update Environment Config.
*   **Details**:
    *   Ensure `ISAAC_VIO_TOPIC` and `ISAAC_DETECTION_TOPIC` defaults in `isaac_ros_bridge.py` match the actual Isaac ROS graph.
    *   Defaults:
        *   VIO: `/visual_slam/tracking/odometry`
        *   Detection: `/yolov8/detections`

### Phase 3: Testing Strategy (Mocked ROS)
**Goal**: Verify the bridge logic without a running Jetson/ROS environment.
*   **Action**: Create `tests/test_isaac_bridge.py`.
*   **Details**:
    *   Use `unittest.mock` to mock `rclpy` and the ROS node.
    *   Test **Message Parsing**: Verify `Detection2DArray` -> `DetectedTarget` conversion.
    *   Test **Exclusion Map**: Verify targets are correctly added and filtered based on the 3D distance logic.
    *   Test **State Updates**: Verify VIO state is correctly updated from `Odometry` messages.

## 4. Verification Steps
1.  **Unit Test**: Run `pytest tests/test_isaac_bridge.py` to verify logic.
2.  **Integration (Hardware)**:
    *   Deploy to Jetson.
    *   Start `isaac_ros_yolov8` launch file.
    *   Run `edge_core`.
    *   Check `/api/isaac/status` returns `available: true`.

## 5. Environment & Dependencies Notes
*   **Python Path**: Python 3.13 is located at `C:\Users\Youssef\AppData\Local\Programs\Python\Python313\python.exe`.
*   **Testing Issues**:
    *   Running `python tests/test_isaac_bridge.py` fails with `ModuleNotFoundError: No module named 'edge_core'`.
    *   Running via `-m unittest` encounters an `ImportError` related to `cv2` (OpenCV) not loading correctly in the Windows environment ("DLL load failed").
*   **Implication**: Local verification of the bridge logic is currently blocked by environmental issues (OpenCV DLLs).
*   **Mitigation**:
    *   Ensure `PYTHONPATH` includes the project root.
    *   The test itself mocks `rclpy` but the `edge_core` imports might trigger other dependencies (like `ultralytics` -> `cv2`) that are failing locally.
    *   The plan remains to deploy to the Jetson (Linux/ARM64) where these dependencies are properly managed via the Docker/system environment.

