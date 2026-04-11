#!/usr/bin/env python3

# Overall Flow:
# camera  -> image_callback -> latest frame 
#                           -> inference loop 
#                           -> preprocess -> onnx -> postprocess
#                           -> draw_boxes -> camera/detected
#                           -> count items -> camera/detections     
#                           -> diff against previous -> camera/diff

# put a filter in btwn that can mask the new item

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
        self.create_subscription(Image, 'camera/image', self.image_callback, qos)

        # topic publishers 
        self.image_pub = self.create_publisher(Image, 'camera/detected', 10)
        self.det_pub = self.create_publisher(String, 'camera/detections', 10)
        self.diff_pub = self.create_publisher(String, 'camera/diff', 10)

        # access ONNX model
        model_dir = get_package_share_directory('model')
        model_path = os.path.join(model_dir, 'onnx', 's_epoch60.onnx')

        # access item database
        price_dir = get_package_share_directory('price')
        json_path = os.path.join(price_dir, 'items', 'grocery_items.json')

        # access class names from either model.yaml or items.json
        class_names_path = os.path.join(price_dir, 'items', 'class_names.json')
        self.class_names = self._load_class_names(class_names_path, json_path)

        opts = onnxruntime.SessionOptions()
        opts.intra_op_num_threads = 4
        opts.inter_op_num_threads = 1
        opts.execution_mode = onnxruntime.ExecutionMode.ORT_SEQUENTIAL
        opts.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = onnxruntime.InferenceSession(
            model_path,
            sess_options=opts,
            providers=['CPUExecutionProvider']
        )
        self.get_logger().info(f'ONNX model has been loaded.')

        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_names = [o.name for o in self.session.get_outputs()]

        output_shapes = [o.shape for o in self.session.get_outputs()]

        self.model_width = 640
        self.model_height = 640

        self.latest_frame = None
        self.frame_processed = True
        self.lock = threading.Lock()

        self.items_in_cart = {}

        self.worker = threading.Thread(target=self.inference_loop, daemon=True)
        self.worker.start()

    # setup

    def _load_class_names(self, class_names_path: str, json_path: str) -> list:
        # Load raw YOLO class names from model export
        try:
            with open(class_names_path, 'r') as f:
                index_map = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load class_names.json: {e}')
            return []

        num_classes = max(int(k) for k in index_map.keys()) + 1
        class_names = [index_map.get(str(i), f'class_{i}') for i in range(num_classes)]
        return class_names

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

    # nms=True export: output shape [N, 6] -> [x1, y1, x2, y2, confidence, class_id]
    # coordinates are in model input space (0..640), scaled back to original frame size
    def postprocess(self, outputs, frame_shape):
        raw = outputs[0]
        if raw.ndim == 3:
            raw = raw[0]  # remove batch dimension if present -> [N, 6]

        if raw is None or len(raw) == 0:
            return [], [], []

        frame_h, frame_w = frame_shape[:2]
        scale_x = frame_w / self.model_width
        scale_y = frame_h / self.model_height

        boxes, classes, confidences = [], [], []

        for det in raw:
            x1, y1, x2, y2, confidence, class_id = det

            if confidence < 0.25: # low confidence threshold to reduce noise
                continue

            boxes.append([
                float(x1) * scale_x,
                float(y1) * scale_y,
                float(x2) * scale_x,
                float(y2) * scale_y,
            ])
            classes.append(int(class_id))
            confidences.append(float(confidence))

        return boxes, classes, confidences

    # annotate frame
    def draw_boxes(self, frame: np.ndarray, boxes, classes, confidences, inference_time_ms: float) -> np.ndarray:
        annotated = frame.copy()

        for box, class_id, conf in zip(boxes, classes, confidences):
            x1, y1, x2, y2 = [int(v) for v in box]
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else f'class_{class_id}'
            label = f'{class_name}: {conf:.2f}%'

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
                infer_start = time.time()
                outputs = self.session.run(self.output_names, {self.input_name: input_data})
                inference_time_ms = (time.time() - infer_start) * 1000.0
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

                detections = []
                for box, class_id, conf in zip(boxes, classes, confidences):
                    detections.append({
                        'class_id': class_id,
                        'class_name': self._class_id_to_name(class_id),
                        'confidence': conf,
                        'box': box,
                    })

                selected_item = None
                
                # If items are removed (and nothing added), select one of the removed items
                if removed:
                    removed_name = list(removed.keys())[0]
                    selected_item = {
                        'class_name': removed_name,
                        'confidence': 0.0,
                        'box': [],
                        'class_id': -1,
                        'is_new': False
                    }
                # If items are added, select one from detections that was added
                elif added:
                    added_names = set(added.keys())
                    for detection in detections:
                        if detection['class_name'] in added_names:
                            selected_item = detection
                            break

                # Fallback to highest confidence detection
                if selected_item is None and detections:
                    selected_item = max(detections, key=lambda d: d['confidence'])

                if selected_item is not None:
                    selected_item['is_new'] = selected_item['class_name'] in (added or {})

                # publish annotated image
                annotated = self.draw_boxes(frame, boxes, classes, confidences, inference_time_ms)
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                )
                self.det_pub.publish(String(data=json.dumps({
                    'classes': classes,
                    'class_names': [self._class_id_to_name(c) for c in classes],
                    'confidences': confidences,
                    'boxes': boxes,
                    'inference_time_ms': inference_time_ms,
                    'selected_item': selected_item,
                    'added': added,
                    'removed': removed,
                })))

                selected_name = selected_item.get('class_name') if selected_item else None
                selected_confidence = (
                    selected_item.get('confidence') if selected_item else None
                )
                selected_confidence_str = (
                    f"{selected_confidence:.2f}" if selected_confidence is not None else "None"
                )
                class_names = [self._class_id_to_name(c) for c in classes]
                detections_str = ', '.join([f'{name}: {conf:.2f}' for name, conf in zip(class_names, confidences)])
                self.get_logger().info(
                    f"All detections: {detections_str} | Inference time: {inference_time_ms} ms"
                )

                if added or removed:
                    self.get_logger().info(f'Changes - Added: {added} | Removed: {removed}')
                else:
                    self.get_logger().info(
                        f'No change detected.'
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