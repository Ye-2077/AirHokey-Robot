from ultralytics import YOLO

# Configure the tracking parameters and run the tracker
model = YOLO('vision/yolo_models/yolov8/yolov8n.pt').to('cuda')
results = model.track(source="vision/samples/sample1.mp4", conf=0.1, iou=0.5, show=True)