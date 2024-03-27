import torch
import matplotlib.pyplot as plt
import numpy as np
import cv2

model = torch.hub.load('ultralytics/yolov5', 'custom' , path = 'vision/models/yolov5_1.pt', force_reload=False).to('cuda')

# cap = cv2.VideoCapture('vision/samples/sample2.mp4')
cap = cv2.VideoCapture('vision/samples/sample1.mp4')
cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
cv2.resizeWindow('YOLO', 800, 600)

while cap.isOpened():
    ret, frame = cap.read()
    results = model(frame)

    cv2.imshow('YOLO', np.squeeze(results.render()))
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()