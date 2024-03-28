import cv2
from vision.detector import Detector



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