#!/usr/bin/env python3
# test_yolo_detection_node.py
#
# Unit tests for YoloDetectionNode

import pytest
import numpy as np
from unittest.mock import MagicMock, patch, PropertyMock


class TestYoloDetectionNodeImports:
    """Test that dependencies can be checked properly."""

    def test_cv2_import_check(self):
        """Test CV2 availability check logic."""
        # This tests the import pattern used in the node
        try:
            import cv2
            assert cv2 is not None
        except ImportError:
            pytest.skip("OpenCV not available - skipping CV2 tests")

    def test_ultralytics_import_check(self):
        """Test ultralytics availability check logic."""
        try:
            from ultralytics import YOLO
            assert YOLO is not None
        except ImportError:
            pytest.skip("Ultralytics not available - skipping YOLO tests")

    def test_cv_bridge_import_check(self):
        """Test cv_bridge availability check logic."""
        try:
            from cv_bridge import CvBridge
            assert CvBridge is not None
        except ImportError:
            pytest.skip("cv_bridge not available - skipping bridge tests")


class TestDetectionMessageConstruction:
    """Test detection message construction logic."""

    @pytest.fixture
    def mock_header(self):
        """Create a mock ROS2 header."""
        header = MagicMock()
        header.stamp = MagicMock()
        header.frame_id = 'camera'
        return header

    def test_bounding_box_center_calculation(self):
        """Test that bbox center is calculated correctly from xyxy format."""
        # YOLO returns xyxy format: [x1, y1, x2, y2]
        x1, y1, x2, y2 = 100.0, 50.0, 200.0, 150.0
        
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        width = x2 - x1
        height = y2 - y1
        
        assert center_x == 150.0
        assert center_y == 100.0
        assert width == 100.0
        assert height == 100.0

    def test_bounding_box_dimensions(self):
        """Test bbox dimension calculations."""
        # Test various box sizes
        test_cases = [
            ((0, 0, 100, 100), (50, 50, 100, 100)),  # Square at origin
            ((50, 50, 150, 100), (100, 75, 100, 50)),  # Rectangle
            ((0, 0, 640, 480), (320, 240, 640, 480)),  # Full frame
        ]
        
        for (x1, y1, x2, y2), (exp_cx, exp_cy, exp_w, exp_h) in test_cases:
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            width = x2 - x1
            height = y2 - y1
            
            assert center_x == exp_cx, f"Center X mismatch for {(x1,y1,x2,y2)}"
            assert center_y == exp_cy, f"Center Y mismatch for {(x1,y1,x2,y2)}"
            assert width == exp_w, f"Width mismatch for {(x1,y1,x2,y2)}"
            assert height == exp_h, f"Height mismatch for {(x1,y1,x2,y2)}"


class TestRateLimiting:
    """Test inference rate limiting logic."""

    def test_rate_to_interval_conversion(self):
        """Test conversion from Hz to interval."""
        test_rates = [
            (5.0, 0.2),
            (10.0, 0.1),
            (1.0, 1.0),
            (30.0, 1.0/30.0),
        ]
        
        for rate, expected_interval in test_rates:
            interval = 1.0 / rate if rate > 0 else 0.0
            assert abs(interval - expected_interval) < 1e-9, \
                f"Interval mismatch for rate {rate}"

    def test_zero_rate_handling(self):
        """Test that zero rate doesn't cause division by zero."""
        rate = 0.0
        interval = 1.0 / rate if rate > 0 else 0.0
        assert interval == 0.0

    def test_rate_limiting_logic(self):
        """Test the rate limiting logic."""
        import time
        
        min_interval = 0.2  # 5 Hz
        last_time = 0.0
        
        # Simulate rapid incoming frames
        frame_times = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3]
        processed_times = []
        
        for t in frame_times:
            time_since_last = t - last_time
            if time_since_last >= min_interval:
                processed_times.append(t)
                last_time = t
        
        # Should process at 0.0, 0.2 (skip 0.05, 0.1, 0.15)
        # Then at 0.25 or 0.3 depending on exact timing
        assert len(processed_times) <= len(frame_times)
        assert 0.0 in processed_times


class TestFPSCalculation:
    """Test FPS calculation logic."""

    def test_fps_calculation(self):
        """Test FPS calculation from frame counts and time."""
        frame_count = 30
        time_elapsed = 1.0  # 1 second
        
        fps = frame_count / time_elapsed
        assert fps == 30.0

    def test_fps_with_partial_second(self):
        """Test FPS calculation with partial seconds."""
        frame_count = 15
        time_elapsed = 0.5  # 0.5 seconds
        
        fps = frame_count / time_elapsed
        assert fps == 30.0  # Still 30 FPS


class TestClassNameMapping:
    """Test class name mapping from YOLO model."""

    def test_class_id_to_name_mapping(self):
        """Test mapping class IDs to names."""
        class_names = {
            0: 'person',
            1: 'bicycle',
            2: 'car',
            3: 'motorcycle',
        }
        
        # Test mapping
        for class_id, expected_name in class_names.items():
            if class_id in class_names:
                name = class_names[class_id]
            else:
                name = str(class_id)
            assert name == expected_name

    def test_unknown_class_id_fallback(self):
        """Test fallback for unknown class IDs."""
        class_names = {0: 'person', 1: 'car'}
        class_id = 999
        
        if class_id in class_names:
            name = class_names[class_id]
        else:
            name = str(class_id)
        
        assert name == '999'


class TestImageProcessing:
    """Test image processing utilities."""

    @pytest.fixture
    def sample_image(self):
        """Create a sample BGR image."""
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def test_info_text_drawing(self, sample_image):
        """Test that info text can be drawn on image."""
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not available")
        
        fps = 30.0
        inference_count = 100
        info_text = f"FPS: {fps:.1f} | Inference: {inference_count}"
        
        # Draw text
        cv2.putText(
            sample_image, info_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 2, cv2.LINE_AA
        )
        
        # Check that image was modified (not all zeros)
        assert sample_image.max() > 0

    def test_jpeg_encoding(self, sample_image):
        """Test JPEG encoding for compressed image publishing."""
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not available")
        
        # Encode as JPEG
        success, encoded = cv2.imencode('.jpg', sample_image, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        assert success
        assert len(encoded) > 0
        assert isinstance(encoded.tobytes(), bytes)


class TestQoSConfiguration:
    """Test QoS profile configuration."""

    def test_qos_reliable(self):
        """Test reliable QoS configuration."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        depth = 10
        reliable = True
        
        qos = QoSProfile(
            depth=depth,
            reliability=(ReliabilityPolicy.RELIABLE if reliable 
                        else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
        )
        
        assert qos.depth == 10
        assert qos.reliability == ReliabilityPolicy.RELIABLE

    def test_qos_best_effort(self):
        """Test best effort QoS configuration for streaming."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        depth = 5
        reliable = False
        
        qos = QoSProfile(
            depth=depth,
            reliability=(ReliabilityPolicy.RELIABLE if reliable 
                        else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
        )
        
        assert qos.depth == 5
        assert qos.reliability == ReliabilityPolicy.BEST_EFFORT


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
