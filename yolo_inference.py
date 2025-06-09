from ultralytics import YOLO

model = YOLO("models/best_keypoint_v3.pt")  # Load a pretrained YOLOv8 model
result = model.predict("input_video/image.png", save=True)  # Predict on an image and save the result
print(result)
