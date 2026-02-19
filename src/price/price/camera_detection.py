from ultralytics import YOLO
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class CameraDetector:

    def __init__(self):
        price_path = get_package_share_directory('price')
        model_path = os.path.join(price_path, 'model', 'my_model.pt')

        try:
            self.model = YOLO(model_path)
        except Exception as e:
            raise RuntimeError(f"Failed to load YOLO model: {e}")
        
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise RuntimeError("Could not open camera")
        except Exception as e:
            raise RuntimeError(f"Camera initialization failed: {e}")

    def detect_item(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                return None, 0.0

            results = self.model(frame)
            detections = results[0].boxes

            if len(detections) == 0:
                return None, 0.0

            # Get highest confidence detection
            best = max(detections, key=lambda b: b.conf[0])
            class_id = int(best.cls[0])
            confidence = float(best.conf[0])

            # Draw result on frame
            annotated = results[0].plot()
            cv2.imshow("YOLO Detection", annotated)
            cv2.waitKey(1)

            return self.model.names[class_id], confidence
        except Exception as e:
            print(f"Error during detection: {e}")
            return None, 0.0

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
