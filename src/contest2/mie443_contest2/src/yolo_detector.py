#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8/v13 and returns the highest confidence detection.
"""

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject
import cv2
import numpy as np
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold
        self.confidence_threshold = 0.5

        # Only these classes are allowed to be returned by the service.
        # Keys are normalized YOLO class names, values are response names.
        self.allowed_classes = {
            'cup': 'cup',
            'motorcycle': 'motorcycle',
            'clock': 'clock',
            'potted plant': 'plant',
            'plant': 'plant',
            'bottle': 'water bottle',
            'water bottle': 'water bottle',
        }
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready')

    def detect_callback(self, request, response):
        """Process image and return highest confidence detection."""
        
        # Decode compressed image
        np_arr = np.frombuffer(request.image.data, np.uint8)
        save_detected_image = request.save_detected_image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is None:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "Failed to decode image"
            return response
        
        # Run YOLO inference
        results = self.model(image, verbose=False, device='cpu')
        boxes = results[0].boxes

        if boxes is None or len(boxes) == 0:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No objects detected"
            return response
        
        # Filter by confidence threshold
        mask = boxes.conf >= self.confidence_threshold
        if not mask.any():
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = f"No detections above confidence {self.confidence_threshold} threshold"
            return response

        # Keep only allowed classes and select the highest-confidence one.
        best_idx = -1
        best_confidence = -1.0
        class_id = -1
        class_name = ""

        for idx in range(len(boxes)):
            confidence = float(boxes.conf[idx])
            if confidence < self.confidence_threshold:
                continue

            detected_class_id = int(boxes.cls[idx])
            detected_class_name = str(self.model.names[detected_class_id]).lower()
            normalized_name = self.allowed_classes.get(detected_class_name)

            if normalized_name is None:
                continue

            if confidence > best_confidence:
                best_idx = idx
                best_confidence = confidence
                class_id = detected_class_id
                class_name = normalized_name

        if best_idx == -1:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No allowed objects detected"
            return response

        confidence = best_confidence

        response.success = True
        response.class_id = class_id
        response.class_name = class_name
        response.confidence = confidence
        response.message = "Detection successful {class_name} with confidence {confidence:.2f}".format(class_name=class_name, confidence=confidence)
        
        if save_detected_image:
            filename = f"/home/turtlebot/ros2_ws/src/contest2/mie443_contest2/yolo_detections/detection_{class_name}_{confidence:.2f}.jpg"
            annotated_image = results[0].plot()
            cv2.imwrite(filename, annotated_image)
            self.get_logger().info(f"Saved detected image to {filename}")
        
        self.get_logger().info(f"Detection: {class_name} (ID: {class_id}) with confidence {confidence:.2f}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()