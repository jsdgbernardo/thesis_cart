#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import threading
import json
import time
import os
import cv2
import numpy as np
import onnxruntime
import yaml

from ament_index_python.packages import get_package_share_directory

MIN_CROP_SIZE = 50

CROP_PADDING = 40

class YoloDetection(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Image, 'camera/image', self.image_callback, qos)

        self.image_pub = self.create_publisher(Image, 'camera/detected', 10)
        self.det_pub = self.create_publisher(String, 'camera/detections', 10)

        model_dir = get_package_share_directory('model')
        price_dir = get_package_share_directory('price')

        model_path = os.path.join(model_dir, 'onnx', 'my_model.onnx')
        metadata_path = os.path.join(model_dir, 'my_model_ncnn_model', 'metadata.yaml')
        json_path = os.path.join(price_dir, 'items', 'grocery_items.json')

        self.class_names = self._load_class_names(metadata_path, json_path)
        self.get_logger().info(f'Loaded {len(self.class_names)} classes: {self.class_names}')

        self.get_logger().info(f'Loading ONNX model: {model_path}')
        self.session = onnxruntime.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']
        )

        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_names = [o.name for o in self.session.get_outputs()]

        self.get_logger().info(f'Model input shape: {self.input_shape}')
        self.get_logger().info(f'Model outputs: {self.output_names}')

        self.model_width = 640
        self.model_height = 640
        self.conf_threshold = 0.7
        self.nms_threshold = 0.45

        self.latest_frame = None
        self.frame_processed = True
        self.lock = threading.Lock()

        # background subtraction: stores the last processed frame so the
        # next triggered frame is subtracted against the most recent stable scene
        self.background_frame = None

        self.worker = threading.Thread(target=self.inference_loop, daemon=True)
        self.worker.start()

    # setup
    def _load_class_names(self, metadata_path: str, json_path: str) -> list:
        try:
            with open(metadata_path, 'r') as f:
                metadata = yaml.safe_load(f)
            index_to_yolo_id = {int(k): v for k, v in metadata['names'].items()}
        except Exception as e:
            self.get_logger().error(f'Failed to load metadata.yaml: {e}')
            return []

        yolo_id_to_name = {}
        try:
            with open(json_path, 'r') as f:
                items_data = json.load(f)
            for value in items_data.values():
                candidates = value if isinstance(value, list) else [value]
                for item in candidates:
                    yolo_id = item.get('yolo_class_id')
                    item_name = item.get('item_name')
                    if yolo_id and item_name:
                        yolo_id_to_name.setdefault(yolo_id, item_name)
        except Exception as e:
            self.get_logger().error(f'Failed to load grocery_items.json: {e}')

        num_classes = max(index_to_yolo_id.keys()) + 1
        return [
            yolo_id_to_name.get(
                index_to_yolo_id.get(i, f'class_{i}'),
                index_to_yolo_id.get(i, f'class_{i}')
            )
            for i in range(num_classes)
        ]

    def _class_id_to_name(self, class_id: int) -> str:
        if class_id < len(self.class_names):
            return self.class_names[class_id]
        return f'class_{class_id}'

    # callback
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self.lock:
            self.latest_frame = frame
            self.frame_processed = False

    # background subtraction
    def get_change_crop(self, frame: np.ndarray):
        """
        Diff the incoming frame against the stored background frame to isolate
        only the region that has changed (the newly added/removed item).

        Returns:
            (crop, offset_x, offset_y)
            - crop:     the sub-image to run YOLO on
            - offset_x: crop's left edge in the original frame
            - offset_y: crop's top edge in the original frame

        Falls back to the full frame if no background exists yet or the
        changed region is too small (likely sensor noise).

        Algorithm:
            1. Per-pixel absolute diff → grayscale
            2. Binary threshold to remove sensor noise (< 25 intensity change)
            3. Dilate + morphological close to merge nearby blobs into one region
            4. Take the largest contour as the change region
            5. Pad its bounding box and crop
        """
        if self.background_frame is None:
            self.get_logger().info('No background yet — using full frame.')
            return frame, 0, 0

        diff = cv2.absdiff(self.background_frame, frame)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

        # suppress noise: only consider pixels that changed by more than 25/255
        _, mask = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)

        # morphological cleanup: dilate to bridge small gaps, then close holes
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().info('No change detected — using full frame.')
            return frame, 0, 0

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < (MIN_CROP_SIZE * MIN_CROP_SIZE):
            self.get_logger().info('Change region too small (noise?) — using full frame.')
            return frame, 0, 0

        bx, by, bw, bh = cv2.boundingRect(largest)
        H, W = frame.shape[:2]

        x1 = max(0, bx - CROP_PADDING)
        y1 = max(0, by - CROP_PADDING)
        x2 = min(W, bx + bw + CROP_PADDING)
        y2 = min(H, by + bh + CROP_PADDING)

        if (x2 - x1) < MIN_CROP_SIZE or (y2 - y1) < MIN_CROP_SIZE:
            return frame, 0, 0

        self.get_logger().info(
            f'Change crop: ({x1},{y1})→({x2},{y2}), size={x2-x1}x{y2-y1}'
        )
        return frame[y1:y2, x1:x2], x1, y1

    # inference pipeline
    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        """BGR frame → (1, 3, 640, 640) float32 in [0, 1]."""
        resized = cv2.resize(frame, (self.model_width, self.model_height))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 255.0
        # (H, W, 3) → (1, 3, H, W)
        return np.transpose(normalized, (2, 0, 1))[np.newaxis, :]

    def postprocess(self, outputs, crop_shape, offset_x: int, offset_y: int):
        """
        YOLOv11 ONNX output: (1, 4 + num_classes, 8400).

        Coordinate path:
            model space (0–640) → crop space → full-frame space
            Full-frame: add offset_x / offset_y from the crop's top-left corner.
        """
        # (1, 4+nc, 8400) → (8400, 4+nc)
        raw = outputs[0][0].T

        crop_h, crop_w = crop_shape[:2]
        scale_x = crop_w / self.model_width
        scale_y = crop_h / self.model_height

        boxes, classes, confidences = [], [], []

        for det in raw:
            cx, cy, w, h = det[:4]
            class_confs = det[4:]

            class_id = int(np.argmax(class_confs))
            confidence = float(class_confs[class_id])

            if confidence < self.conf_threshold:
                continue

            # model space → crop space, then shift to full-frame coords
            x1 = (cx - w / 2) * scale_x + offset_x
            y1 = (cy - h / 2) * scale_y + offset_y
            x2 = (cx + w / 2) * scale_x + offset_x
            y2 = (cy + h / 2) * scale_y + offset_y

            boxes.append([x1, y1, x2, y2])
            classes.append(class_id)
            confidences.append(confidence)

        if not boxes:
            return [], [], []

        boxes_arr = np.array(boxes, dtype=np.float32)
        confs_arr = np.array(confidences, dtype=np.float32)
        keep = self.nms(boxes_arr, confs_arr, self.nms_threshold)

        return (
            boxes_arr[keep].tolist(),
            [classes[i] for i in keep],
            [confidences[i] for i in keep],
        )

    def nms(self, boxes: np.ndarray, scores: np.ndarray, threshold: float) -> list:
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(int(i))

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            inter_w = np.maximum(0.0, xx2 - xx1)
            inter_h = np.maximum(0.0, yy2 - yy1)
            inter = inter_w * inter_h

            iou = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(iou <= threshold)[0] + 1]

        return keep

    def draw_boxes(self, frame: np.ndarray, boxes, classes, confidences) -> np.ndarray:
        annotated = frame.copy()

        for box, class_id, conf in zip(boxes, classes, confidences):
            x1, y1, x2, y2 = [int(v) for v in box]
            name = self._class_id_to_name(class_id)
            label = f'{name}: {conf:.2f}'

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(annotated, (x1, y1 - th - 10), (x1 + tw, y1), (0, 255, 0), -1)
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        return annotated

    # main loop
    def inference_loop(self):
        while rclpy.ok():
            with self.lock:
                if self.latest_frame is None or self.frame_processed:
                    frame = None
                else:
                    frame = self.latest_frame.copy()
                    self.frame_processed = True

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                # 1. Isolate the region that changed vs the last stable scene
                crop, offset_x, offset_y = self.get_change_crop(frame)

                # 2. Run YOLO only on the changed crop
                input_data = self.preprocess(crop)
                outputs = self.session.run(self.output_names, {self.input_name: input_data})

                # 3. Map detections back to full-frame coordinates
                boxes, classes, confidences = self.postprocess(
                    outputs, crop.shape, offset_x, offset_y
                )

                # 4. Draw boxes on the full frame for visual context
                annotated = self.draw_boxes(frame, boxes, classes, confidences)
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                )

                detections_data = {
                    'classes': classes,
                    'class_names': [self._class_id_to_name(c) for c in classes],
                    'confidences': confidences,
                    'boxes': boxes,
                }
                self.det_pub.publish(String(data=json.dumps(detections_data)))

                if classes:
                    self.get_logger().info(
                        f'Detected {len(classes)} item(s): '
                        + ', '.join(
                            f'{self._class_id_to_name(c)} ({conf:.2f})'
                            for c, conf in zip(classes, confidences)
                        )
                    )
                else:
                    self.get_logger().info('No detections.')

                # 5. Update background AFTER publishing — the next triggered frame
                #    will diff against the scene as it looks NOW (item present/absent)
                with self.lock:
                    self.background_frame = frame.copy()

            except Exception as e:
                self.get_logger().error(f'Inference error: {e}', exc_info=True)


def main():
    rclpy.init()
    node = YoloDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()