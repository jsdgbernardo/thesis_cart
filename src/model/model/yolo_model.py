#!/usr/bin/env python3

# Overall Flow:
# camera  -> image_callback -> latest frame 
#                           -> inference loop 
#                           -> preprocess -> onnx -> postprocess -> nms
#                           -> draw_boxes -> camera/detected
#                           -> count items -> camera/detections     
#                           -> diff against previous -> camera/diff

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

class YoloModel(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        # handling heavy interference 
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # topic subscribers 
        self.create_subscription(Image, 'camera/image', self.image_callback, qos) # latest frame

        # topic publishers 
        self.image_pub = self.create_publisher(Image, 'camera/detected', 10) # annotated image with detections
        self.det_pub = self.create_publisher(String, 'camera/detections', 10) # raw detection data: classes, confidences, boxes
        self.diff_pub = self.create_publisher(String, 'camera/diff', 10) # what's added/removed since last inference

        # access ONNX model
        model_dir = get_package_share_directory('model')
        model_path = os.path.join(model_dir, 'onnx', 'my_model.onnx')
        metadata_path = os.path.join(model_dir, 'my_model_ncnn_model', 'metadata.yaml')

        # acces item database
        price_dir = get_package_share_directory('price')
        json_path = os.path.join(price_dir, 'items', 'grocery_items.json')

        # acces class names from either model.yaml or items.json
        self.class_names = self._load_class_names(metadata_path, json_path)
        self.get_logger().info(f'Loaded {len(self.class_names)} classes: {self.class_names}')

        self.session = onnxruntime.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']
        )
        self.get_logger().info(f'ONNX model has been loaded.')

        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_names = [o.name for o in self.session.get_outputs()]

        self.get_logger().info(f'Model input shape: {self.input_shape}')
        self.get_logger().info(f'Model outputs: {self.output_names}')

        self.model_width = 640
        self.model_height = 640

        self.latest_frame = None
        self.frame_processed = True
        self.lock = threading.Lock()

        self.items_in_cart = {}

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

    # callbacks
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self.lock:
            self.latest_frame = frame
            self.frame_processed = False

    # inference pipeline

    # BGR -> 640x640 RGB, normalized, NCHW format
    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        resized = cv2.resize(frame, (self.model_width, self.model_height))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 255.0
        return np.transpose(normalized, (2, 0, 1))[np.newaxis, :]

    # 
    def postprocess(self, outputs, frame_shape):
        raw = outputs[0][0].T
        frame_h, frame_w = frame_shape[:2]
        scale_x = frame_w / self.model_width
        scale_y = frame_h / self.model_height

        boxes, classes, confidences = [], [], []

        for det in raw:
            cx, cy, w, h = det[:4]
            class_confs = det[4:]
            class_id = int(np.argmax(class_confs))
            confidence = float(class_confs[class_id])

            # discard weak detections immediately
            if confidence < 0.7:
                continue

            x1 = (cx - w / 2) * scale_x
            y1 = (cy - h / 2) * scale_y
            x2 = (cx + w / 2) * scale_x
            y2 = (cy + h / 2) * scale_y

            boxes.append([x1, y1, x2, y2])
            classes.append(class_id)
            confidences.append(confidence)

        if not boxes:
            return [], [], []

        boxes_arr = np.array(boxes, dtype=np.float32)
        confs_arr = np.array(confidences, dtype=np.float32)
        keep = self.nms(boxes_arr, confs_arr, 0.45)

        return (
            boxes_arr[keep].tolist(),
            [classes[i] for i in keep],
            [confidences[i] for i in keep],
        )

    # Non-Maximum Suppression to remove overlapping boxes
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
            
            # intersection over union
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(iou <= threshold)[0] + 1]

        return keep

    # annotate frame
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
                input_data = self.preprocess(frame)
                outputs = self.session.run(self.output_names, {self.input_name: input_data})
                boxes, classes, confidences = self.postprocess(outputs, frame.shape)

                # counts how many of each item was detected in the frame
                new_state = {}
                for class_id in classes:
                    name = self._class_id_to_name(class_id)
                    new_state[name] = new_state.get(name, 0) + 1

                # difference against previous list of items
                all_keys = set(self.items_in_cart) | set(new_state)
                added, removed = {}, {}
                for name in all_keys:
                    old_count = self.items_in_cart.get(name, 0)
                    new_count = new_state.get(name, 0)
                    if new_count > old_count:
                        added[name] = new_count - old_count
                    elif new_count < old_count:
                        removed[name] = old_count - new_count

                self.items_in_cart = new_state

                # publish differences
                diff_data = {'added': added, 'removed': removed}
                self.diff_pub.publish(String(data=json.dumps(diff_data)))

                # publish annotated image
                annotated = self.draw_boxes(frame, boxes, classes, confidences)
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                )
                self.det_pub.publish(String(data=json.dumps({
                    'classes': classes,
                    'class_names': [self._class_id_to_name(c) for c in classes],
                    'confidences': confidences,
                    'boxes': boxes,
                })))

                if added or removed:
                    self.get_logger().info(f'Added: {added} | Removed: {removed}')
                else:
                    self.get_logger().info(
                        f'No change detected. Current State: {self.items_in_cart}'
                    )

            except Exception as e:
                self.get_logger().error(f'Inference error: {e}', exc_info=True)


def main():
    rclpy.init()
    node = YoloModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()