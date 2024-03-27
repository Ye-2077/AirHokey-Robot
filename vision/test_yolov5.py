import torch
import matplotlib.pyplot as plt
import numpy as np
import cv2

model = torch.hub.load('ultralytics/yolov5', 'custom' , path = 'vision/models/yolov5_1.pt', force_reload=False).to('cuda')

cap = cv2.VideoCapture('vision/samples/sample1.mp4')
cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
cv2.resizeWindow('YOLO', 800, 600)

while cap.isOpened():
    ret, frame = cap.read()
    results = model(frame)
    puck_objects = results.pandas().xyxy[0][results.pandas().xyxy[0]['name'] == 'puck']
    # print(puck_objects)

    if not puck_objects.empty:
        for index, puck in puck_objects.iterrows():
            # Extract the bounding box coordinates
            puck_xmin, puck_ymin, puck_xmax, puck_ymax = puck['xmin'], puck['ymin'], puck['xmax'], puck['ymax']
            print(f'Puck: x: {puck_xmin} to {puck_xmax}')
            print(f'Puck: y: {puck_ymin} to {puck_ymax}')
    else:
        print('No puck detected')

    cv2.imshow('YOLO', np.squeeze(results.render()))
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()