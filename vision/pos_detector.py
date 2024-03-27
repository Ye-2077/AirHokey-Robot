import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch

def detect_pos(result, target='puck'):
    '''
    Detect the position in the frame
    - result: the result of the YOLOv5 model
    - return: the exact position in the frame
    '''
    puck_objects = result.pandas().xyxy[0][result.pandas().xyxy[0]['name'] == target]
    if not puck_objects.empty:
        for index, puck in puck_objects.iterrows():
            puck_xmin, puck_ymin, puck_xmax, puck_ymax = puck['xmin'], puck['ymin'], puck['xmax'], puck['ymax']
            # print(f'Puck: x: {puck_xmin} to {puck_xmax}')
            # print(f'Puck: y: {puck_ymin} to {puck_ymax}')
            puck_x = (puck_xmin + puck_xmax) / 2
            puck_y = (puck_ymin + puck_ymax) / 2
            detect_flag = True
    else:
        # print('No puck detected')
        puck_x = None
        puck_y = None
        detect_flag = False

    return puck_x, puck_y, detect_flag

def init_kalman(x,y):
    '''
    Initialize the Kalman filter
    - x: the initial x-coordinate
    - y: the initial y-coordinate
    '''
    kalman = cv2.KalmanFilter(4,2)
    kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
    kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
    kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
    kalman.statePre = np.array([[x], [y], [0], [0]], np.float32)
    kalman.errorCovPre = np.eye(4, dtype=np.float32)
    return kalman

def update_kalman(kalman, x, y):
    '''
    update the Kalman filter
    - kalman: the Kalman filter
    '''
    kalman.correct(np.array([[x], [y]], np.float32))
    prediction = kalman.predict()
    return prediction[0], prediction[1]