from ultralytics import YOLO
import cv2
import pickle
import pandas as pd

class CourtLineDetector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def predict(self,img):
        results = self.model(img)[0]
        keypoints = results[0].keypoints.xy
        return keypoints        

    def draw_keypoints(self, image, keypoints_tensor):
        keypoints = keypoints_tensor.squeeze(0).tolist()  # Từ (1, 12, 2) => (12, 2)
        for i, (x, y) in enumerate(keypoints):
            x, y = int(x), int(y)
            cv2.circle(image, (x, y), radius=5, color=(0, 0, 225), thickness=-1)  # Vẽ chấm đỏ
            cv2.putText(image, str(i), (x + 5, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 225) , 1)  # Hiện số thứ tự
        return image
    
    def draw_keypoint_on_video(self, video_frames, keypoints):
        output_frames = []
        for frame in video_frames:
            frame = self.draw_keypoints(frame, keypoints)
            output_frames.append(frame)
        return output_frames        