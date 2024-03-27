import torch
import matplotlib.pyplot as plt
import numpy as np
import cv2
import math

from vision.pos_detector import *

model = torch.hub.load('ultralytics/yolov5', 'custom' , path = 'vision/models/yolov5_1.pt', force_reload=False).to('cuda')

cap = cv2.VideoCapture('vision/samples/sample1.mp4')
cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
cv2.resizeWindow('YOLO', 800, 600)
fps = cap.get(cv2.CAP_PROP_FPS)
print(f'FPS: {fps}')

init_ret, init_frame = cap.read()
if not init_ret:
    print('Failed to read video')
    cap.release()
    cv2.destroyAllWindows()
    exit()

init_results = model(init_frame)
init_puck_x, init_puck_y, detect_flag = detect_pos(init_results)
print(f'Initial Puck: x: {init_puck_x}, y: {init_puck_y}')

kalman = init_kalman(init_puck_x, init_puck_y)

trajectory = []
predicy_trajectory_length = 20

table_left = 250
table_right = 1600
table_up = 140
table_down = 930

while cap.isOpened():
    ret, frame = cap.read()
    results = model(frame)
    cv2.rectangle(frame, (table_left, table_up), (table_right, table_down), (0, 255, 0), thickness=5, lineType=None, shift=None)

    # detect the position of the puck
    puck_x, puck_y, detect_flag = detect_pos(results)
    
    # update the Kalman filter and predict the next position
    if detect_flag:
        predict_x, predict_y = update_kalman(kalman, puck_x, puck_y)
    else:
        prediction = kalman.predict()
        predict_x, predict_y = prediction[0], prediction[1]



    trajectory.append((int(predict_x), int(predict_y)))
    if len(trajectory) > 10:
        trajectory.pop(0)
    for point in trajectory:
        cv2.circle(frame, point, 3, (255, 255, 0), -1)
    for i in range(1, len(trajectory)):
        cv2.line(frame, trajectory[i - 1], trajectory[i], (255, 255, 0), 2)

    # calculate the speed vector
    if len(trajectory) > 1:
        dx = np.mean([trajectory[i][0] - trajectory[i-1][0] for i in range(1, len(trajectory))])
        dy = np.mean([trajectory[i][1] - trajectory[i-1][1] for i in range(1, len(trajectory))])
    else:
        dx = dy = 0


    speed_magnitude = 5
    if dx != 0 or dy != 0:
        unit_dx = dx / math.sqrt(dx ** 2 + dy ** 2)
        unit_dy = dy / math.sqrt(dx ** 2 + dy ** 2)
    else:
        unit_dx, unit_dy = 0, 0

    future_trajectory = []
    last_point = trajectory[-1]
    for i in range(1, predicy_trajectory_length + 1):
        next_x = int(last_point[0] + unit_dx * speed_magnitude * i)
        next_y = int(last_point[1] + unit_dy * speed_magnitude * i)
        
        # detect the border and bounce back
        if next_x <= table_left or next_x >= table_right:
            unit_dx *= -1
            next_x = int(last_point[0] + unit_dx * speed_magnitude * i)
        
        if next_y <= table_up or next_y >= table_down:
            unit_dy *= -1
            next_y = int(last_point[1] + unit_dy * speed_magnitude * i)
        
        future_trajectory.append((next_x, next_y))
        last_point = (next_x, next_y)

    for i in range(1, len(future_trajectory)):
        cv2.line(frame, future_trajectory[i - 1], future_trajectory[i], (0, 0, 255), 2)


    
    cv2.imshow('YOLO', frame)
    # draw the puck position
    # cv2.imshow('YOLO', np.squeeze(results.render()))
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()