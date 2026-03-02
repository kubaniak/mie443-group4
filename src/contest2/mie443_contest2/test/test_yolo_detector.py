import pytest
import cv2
import numpy as np
import os
import sys

# Instead of mocking rclpy at the module level which messes with class creation,
# we will mock the Node class and YOLO model before importing.
from unittest.mock import MagicMock, patch

class DummyNode:
    def __init__(self, name):
        pass
    def create_service(self, *args, **kwargs):
        pass
    def get_logger(self):
        logger = MagicMock()
        return logger

# Mock rclpy modules
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.node'].Node = DummyNode
sys.modules['mie443_contest2'] = MagicMock()
sys.modules['mie443_contest2.srv'] = MagicMock()

# Now we can import
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from yolo_detector import YoloDetectorNode

class MockRequest:
    def __init__(self, image_data, save_detected_image=False):
        self.image = MagicMock()
        self.image.data = image_data
        self.save_detected_image = save_detected_image

class MockResponse:
    def __init__(self):
        self.success = False
        self.class_id = -1
        self.class_name = ""
        self.confidence = 0.0
        self.message = ""

def create_mock_image(color=(255, 255, 255)):
    """Create a simple colored mock image encoded as bytes."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = color
    # Encode to match what the service expects
    _, buffer = cv2.imencode('.jpg', img)
    return buffer.tobytes()

@patch('yolo_detector.YOLO')
def test_yolo_detector_init(mock_yolo_class):
    """Test that the node initializes correctly with target classes."""
    node = YoloDetectorNode()
    assert "cup" in node.target_classes
    assert "bottle" in node.target_classes
    assert node.confidence_threshold == 0.5

@patch('yolo_detector.YOLO')
def test_yolo_detect_invalid_image(mock_yolo_class):
    """Test behavior with invalid image data."""
    node = YoloDetectorNode()
    request = MockRequest(b"invalid_data")
    response = MockResponse()

    result = node.detect_callback(request, response)

    assert result.success == False
    assert result.class_id == -1
    assert "Failed to decode image" in result.message

@patch('yolo_detector.YOLO')
def test_yolo_detect_empty_image(mock_yolo_class):
    """Test behavior with a blank image where no objects should be found."""
    # Setup mock YOLO instance
    mock_yolo_instance = MagicMock()
    mock_yolo_class.return_value = mock_yolo_instance

    node = YoloDetectorNode()

    # Mock the YOLO model to return no detections
    mock_results = MagicMock()
    mock_results.boxes = []
    # When node.model(image) is called, return [mock_results]
    mock_yolo_instance.return_value = [mock_results]

    request = MockRequest(create_mock_image())
    response = MockResponse()

    result = node.detect_callback(request, response)

    # Empty image should not contain any target objects with high confidence
    assert result.success == False
    assert result.class_id == -1
    assert "No target objects detected" in result.message

@patch('yolo_detector.YOLO')
def test_yolo_detect_save_image(mock_yolo_class, tmpdir):
    """Test that the callback doesn't crash when save_detected_image is True."""
    # Setup mock YOLO instance
    mock_yolo_instance = MagicMock()
    mock_yolo_class.return_value = mock_yolo_instance

    node = YoloDetectorNode()

    original_cwd = os.getcwd()
    os.chdir(str(tmpdir))

    try:
        # Mock the YOLO model to return no detections
        mock_results = MagicMock()
        mock_results.boxes = []
        mock_yolo_instance.return_value = [mock_results]

        request = MockRequest(create_mock_image(), save_detected_image=True)
        response = MockResponse()

        result = node.detect_callback(request, response)

        # In an empty image, it shouldn't save because success is False
        assert result.success == False
        assert not os.path.exists("detected_manipulable_object.jpg")
    finally:
        os.chdir(original_cwd)

@patch('yolo_detector.YOLO')
def test_yolo_detect_valid_object(mock_yolo_class, tmpdir):
    """Test behavior when a valid target object is detected."""
    # Setup mock YOLO instance
    mock_yolo_instance = MagicMock()
    mock_yolo_class.return_value = mock_yolo_instance

    node = YoloDetectorNode()

    original_cwd = os.getcwd()
    os.chdir(str(tmpdir))

    try:
        # Mock a box detection
        mock_box = MagicMock()
        mock_box.cls = [MagicMock(item=lambda: 41)]
        mock_box.conf = [MagicMock(item=lambda: 0.85)]
        mock_box.xyxy = [np.array([10, 10, 100, 100])]

        mock_results = MagicMock()
        mock_results.boxes = [mock_box]

        mock_yolo_instance.return_value = [mock_results]
        mock_yolo_instance.names = {41: "cup"}

        request = MockRequest(create_mock_image(), save_detected_image=True)
        response = MockResponse()

        result = node.detect_callback(request, response)

        assert result.success == True
        assert result.class_id == 41
        assert result.class_name == "cup"
        assert result.confidence == 0.85
        assert "Detected cup with confidence 0.85" in result.message

        # Since save_detected_image was True and detection succeeded, it should save the image
        assert os.path.exists("detected_manipulable_object.jpg")
    finally:
        os.chdir(original_cwd)
