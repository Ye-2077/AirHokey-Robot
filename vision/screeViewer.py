import cv2
import numpy as np
from PIL import ImageGrab
import torch

model = torch.hub.load('ultralytics/yolov5', 'custom' , path = 'vision/yolov5-master/runs/train/yolo_puck_1/weights/best.pt', force_reload=True).to('cuda')

def grab_screen():
    screen = np.array(ImageGrab.grab(bbox=(80, 0, 1200, 1080)))
    screen = cv2.cvtColor(screen, cv2.COLOR_BGR2RGB)
    cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('YOLO', 800, 600)
    return screen

while True:
    screen = grab_screen()
    results = model(screen)
    cv2.imshow('YOLO', np.squeeze(results.render()))
    if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break
