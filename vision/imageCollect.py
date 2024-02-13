import uuid
import os
import time
import cv2

IMAGE_PATH = os.path.join('data', 'images/sample1')
# labels = ['puck', 'player1', 'player2', 'table']
num_imgs = 50

## collect images
cap = cv2.VideoCapture("../vision/samples/sample1.mp4")

# for label in labels:
for img_num in range(num_imgs):
    # print(f'Collecting image {img_num} for {label}')
    
    ret, frame = cap.read()
    img_name = os.path.join(IMAGE_PATH, str(uuid.uuid1())+'.jpg')
    cv2.imwrite(img_name, frame)
    cv2.imshow('frame', frame)
    time.sleep(2)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()