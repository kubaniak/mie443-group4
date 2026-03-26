import sys
from unittest.mock import MagicMock

# Mock imports properly
class MockNode:
    def __init__(self, name):
        pass
    def create_service(self, *args, **kwargs):
        pass
    def get_logger(self):
        class MockLogger:
            def info(self, msg):
                pass
        return MockLogger()

class MockRclpy:
    def init(self, args=None):
        pass
    def spin(self, node):
        pass
    def shutdown(self):
        pass

mock_rclpy = MockRclpy()
mock_rclpy.node = MagicMock()
mock_rclpy.node.Node = MockNode

sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy.node
sys.modules['mie443_contest2'] = MagicMock()
sys.modules['mie443_contest2.srv'] = MagicMock()
sys.modules['cv2'] = MagicMock()
sys.modules['numpy'] = MagicMock()
sys.modules['ultralytics'] = MagicMock()

import src.contest2.mie443_contest2.src.yolo_detector as yolo_detector

def test_yolo_detector_init():
    class MockModel:
        def __init__(self):
            self.names = {0: 'cup', 1: 'motorcycle', 2: 'clock', 3: 'potted plant', 4: 'bottle', 5: 'person', 6: 'car', 7: 'water bottle', 8: 'plant'}

    yolo_detector.YOLO = MagicMock(return_value=MockModel())
    node = yolo_detector.YoloDetectorNode()

    # Check if cache is built correctly
    assert node._class_id_to_normalized_name == {
        0: 'cup',
        1: 'motorcycle',
        2: 'clock',
        3: 'plant',
        4: 'water bottle',
        7: 'water bottle',
        8: 'plant'
    }

if __name__ == "__main__":
    test_yolo_detector_init()
    print("Test passed!")
