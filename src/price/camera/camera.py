import cv2
import ultralytics

class CameraDetector:
 
    def __init__(self):
        self.stream_url = "http://172.57.181.53:8080" # phone ip webcam, might change to actual webcam nlng
        
        try:
            self.cap = cv2.VideoCapture(self.stream_url)
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open video stream: {self.stream_url}')
                return
        except Exception as e:
            raise RuntimeError(f"Camera initialization failed: {e}")
        
        self.time = self.create_timer(0.25, self.detect_item) # 4 fps

        # model path: should convert from pt to ncnn
    
    def detect_item(self):
        # try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                return None
            return frame 

        #     results = self.model(frame)
        #     detections = results[0].boxesz  

        #     if len(detections) == 0:
        #         return None, 0.0

        #     # get highest confidence detection
        #     best = max(detections, key=lambda b: b.conf[0])
        #     class_id = int(best.cls[0])
        #     confidence = float(best.conf[0])

        #     # draw result on frame
        #     annotated = results[0].plot()
        #     cv2.imshow("YOLO Detection", annotated)
        #     cv2.waitKey(1)

        #     return self.model.names[class_id], confidence
        
        # except Exception as e:
        #     print(f"Error during detection: {e}")
        #     return None, 0.0

def main(args=None):
    node = CameraDetector()
    node.cap.release()
    cv2.destroyAllWindows()