## uncomment the following 3 line if you are using Windows
import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath
#########################################################



import cv2
import matplotlib.pyplot as plt
import math
import numpy as np
import torch

class Detector:
    def __init__(self, 
                 model_path,
                 tabel=(250, 1600, 140, 930),
                 device='cuda'):
        self.device = device
        self.model = torch.hub.load('ultralytics/yolov5', 'custom' , path = model_path, force_reload=False).to(self.device)
        self.kalman = None
        self.trajectory = []
        self.table = tabel

    def detect_pos(self, result, target):
        '''
        Detect the position in the frame
        - result: the result of the YOLOv5 model
        - return: the exact position in the frame
        '''
        puck_objects = result.pandas().xyxy[0][result.pandas().xyxy[0]['name'] == target]
        if not puck_objects.empty:
            for index, puck in puck_objects.iterrows():
                puck_xmin, puck_ymin, puck_xmax, puck_ymax = puck['xmin'], puck['ymin'], puck['xmax'], puck['ymax']
                puck_x = (puck_xmin + puck_xmax) / 2
                puck_y = (puck_ymin + puck_ymax) / 2
                detect_flag = True
        else:
            puck_x = None
            puck_y = None
            detect_flag = False

        return puck_x, puck_y, detect_flag
    
    def init_kalman(self, x, y):
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
    
    def update_kalman(self, kalman, x, y):
        '''
        update the Kalman filter
        - kalman: the Kalman filter
        '''
        kalman.correct(np.array([[x], [y]], np.float32))
        prediction = kalman.predict()
        return prediction[0], prediction[1]
    
    def detect(self, frame, 
               target='puck', 
               render_trajectory=True):
        '''
        Detect the position of the target in the frame
        - frame: the input frame
        - target: the target object to detect
        '''
        results = self.model(frame)
        cv2.rectangle(frame, (self.table[0], self.table[2]), (self.table[1], self.table[3]),
                      (0, 255, 0), thickness=5, lineType=None, shift=None)

        # detect the position of the puck
        puck_x, puck_y, detect_flag = self.detect_pos(results, target=target)

        # update the Kalman filter and predict the next position
        if detect_flag:
            predict_x, predict_y = self.update_kalman(self.kalman, puck_x, puck_y)
        else:
            prediction = self.kalman.predict()
            predict_x, predict_y = prediction[0], prediction[1]

        # draw the trajectory
        if render_trajectory:
            self.trajectory.append((int(predict_x), int(predict_y)))
            if len(self.trajectory) > 10:
                self.trajectory.pop(0)
            for point in self.trajectory:
                cv2.circle(frame, point, 3, (255, 255, 0), -1)
            for i in range(1, len(self.trajectory)):
                cv2.line(frame, self.trajectory[i - 1], self.trajectory[i], (255, 255, 0), 2)
            
        
        return frame, self.trajectory

    def predict_trajectory(self, 
                           frame, 
                           render_trajectory=True):
        '''
        Predict the future trajectory
        - frame: the input frame
        - render_trajectory: whether to render the trajectory
        '''
        if(len(self.trajectory) > 1):
            dx = np.mean([self.trajectory[i][0] - self.trajectory[i-1][0] for i in range(1, len(self.trajectory))])
            dy = np.mean([self.trajectory[i][1] - self.trajectory[i-1][1] for i in range(1, len(self.trajectory))])
            unit_dx = dx / math.sqrt(dx ** 2 + dy ** 2)
            unit_dy = dy / math.sqrt(dx ** 2 + dy ** 2)
        else:
            dx,dy = 0,0
            unit_dx, unit_dy = 0, 0
        
        future_trajectory = []
        last_point = self.trajectory[-1]
        speed_magnitude = 10
        for i in range(1, 11):
            next_x = int(last_point[0] + unit_dx * speed_magnitude * i)
            next_y = int(last_point[1] + unit_dy * speed_magnitude * i)
            
            # detect the border and bounce back
            if next_x <= self.table[0] or next_x >= self.table[1]:
                unit_dx *= -1
                next_x = int(last_point[0] + unit_dx * speed_magnitude * i)
            
            if next_y <= self.table[2] or next_y >= self.table[3]:
                unit_dy *= -1
                next_y = int(last_point[1] + unit_dy * speed_magnitude * i)
            
            future_trajectory.append((next_x, next_y))
            last_point = (next_x, next_y)
        
        if render_trajectory:
            cv2.circle(frame, future_trajectory[-1], 10, (0, 0, 255), -1)
            for i in range(1, len(future_trajectory)):
                cv2.line(frame, future_trajectory[i - 1], future_trajectory[i], (0, 0, 255), 4)
        
        return frame, future_trajectory

    

if __name__ == '__main__':
    cap = cv2.VideoCapture('vision/samples/sample1.mp4')
    tabel=(250, 1600, 140, 930) # need to be adjusted handly
    fps = cap.get(cv2.CAP_PROP_FPS)
    detector = Detector('vision/models/yolov5_1.pt', tabel=tabel, device='cuda')
    
    # Initialize the Kalman filter
    init_puck_x, init_puck_y = detector.detect_pos(detector.model(cap.read()[1]), target='puck')[:2]
    detector.kalman = detector.init_kalman(init_puck_x, init_puck_y)
    
    cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('YOLO', 800, 600)
    while cap.isOpened():
        ret, frame = cap.read()
        frame, trajectory = detector.detect(frame)
        frame, future_trajectory = detector.predict_trajectory(frame)

        cv2.imshow('YOLO', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    