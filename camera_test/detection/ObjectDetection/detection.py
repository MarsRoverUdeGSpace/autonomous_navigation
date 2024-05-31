import cv2
import numpy as np
from ultralytics import YOLO

class Detection:
    def __init__(self):
        self.yolo_model = YOLO("hammer_bottle.pt")
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(3, 640)
        self.video_capture.set(4, 640)
        self.stopped = False

    def start(self):
        while not self.stopped:
            ret, frame = self.video_capture.read()
            if ret:
                self.process_frame(frame)
        self.video_capture.release()

    def process_frame(self, frame):
        detection_results = self.yolo_model.predict(frame, imgsz=640, conf=0.83)
        annotated_image = detection_results[0].plot()
        cv2.imshow("YOLOv5 Detection", annotated_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop()

    def stop(self):
        self.stopped = True
        cv2.destroyAllWindows()