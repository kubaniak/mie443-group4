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
        
        # Target classes for the contest
        self.target_classes = ["cup", "bottle", "clock", "potted plant", "motorcycle"]

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

        ### YOUR CODE HERE ###
        best_conf = 0.0
        best_class_id = -1
        best_class_name = ""

        # Dictionary mapping COCO class IDs to names
        names = self.model.names

        for box in boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())

            if cls_id in names:
                class_name = names[cls_id]

                # Check if it's a target class and meets confidence threshold
                if class_name in self.target_classes and conf >= self.confidence_threshold:
                    if conf > best_conf:
                        best_conf = conf
                        best_class_id = cls_id
                        best_class_name = class_name

        if best_class_id != -1:
            # We found a valid object
            response.success = True
            response.class_id = best_class_id
            response.class_name = best_class_name
            response.confidence = best_conf
            response.message = f"Detected {best_class_name} with confidence {best_conf:.2f}"

            self.get_logger().info(response.message)

            # Save annotated image if requested
            if request.save_detected_image:
                annotated_img = results[0].plot()
                save_path = "detected_manipulable_object.jpg"
                cv2.imwrite(save_path, annotated_img)
                self.get_logger().info(f"Saved annotated image to {save_path}")

        else:
            # No valid object found
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No target objects detected above confidence threshold"

            self.get_logger().warn(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
