import unittest
from unittest.mock import MagicMock, patch
import sys
import os

# Mock rclpy before importing the bridge
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.qos"] = MagicMock()
sys.modules["nav_msgs.msg"] = MagicMock()
sys.modules["vision_msgs.msg"] = MagicMock()

# Import the module under test
from edge_core.isaac_ros_bridge import IsaacROSBridge, DetectedTarget, VIOState

class TestIsaacROSBridge(unittest.TestCase):
    def setUp(self):
        self.bridge = IsaacROSBridge(
            vio_topic="/test/vio",
            detection_topic="/test/det",
            exclusion_radius=0.5
        )

    def test_initial_state(self):
        status = self.bridge.get_status()
        self.assertFalse(status["running"])
        self.assertEqual(status["vio_count"], 0)
        self.assertEqual(status["detection_count"], 0)

    def test_exclusion_logic(self):
        # Add a target at (10, 10, 0)
        self.bridge.add_to_exclusion_map(10.0, 10.0, 0.0, radius=1.0, target_id="t1")
        
        # Test exact hit
        excluded, tid = self.bridge.is_excluded(10.0, 10.0, 0.0)
        self.assertTrue(excluded)
        self.assertEqual(tid, "t1")
        
        # Test within radius (0.9m distance)
        excluded, tid = self.bridge.is_excluded(10.9, 10.0, 0.0)
        self.assertTrue(excluded)
        
        # Test outside radius (1.1m distance)
        excluded, tid = self.bridge.is_excluded(11.1, 10.0, 0.0)
        self.assertFalse(excluded)

    def test_detection_filtering(self):
        # Add exclusion
        self.bridge.add_to_exclusion_map(5.0, 5.0, 0.0, radius=1.0)
        
        # Create dummy detections
        det_good = DetectedTarget(0, "cls", 1, 0.9, 0, 0, 10, 10, world_x=0.0, world_y=0.0, world_z=0.0)
        det_bad = DetectedTarget(0, "cls", 1, 0.9, 0, 0, 10, 10, world_x=5.0, world_y=5.0, world_z=0.0)
        
        filtered = self.bridge._filter_detections([det_good, det_bad])
        
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0], det_good)

if __name__ == "__main__":
    unittest.main()
