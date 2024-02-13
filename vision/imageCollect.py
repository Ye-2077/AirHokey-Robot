import uuid
import os
import time
import cv2


VIDEO_PATH = "vision/samples/sample2.mp4"
IMAGE_PATH = os.path.join('vision/data', 'images/sample2')
num_imgs_per_video = 50

cap = cv2.VideoCapture(VIDEO_PATH)

# gain the basic information of the video
num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps = cap.get(cv2.CAP_PROP_FPS)

# calculate the interval of frames to capture
frame_interval = max(1, num_frames // num_imgs_per_video)

img_num = 0
while True:
    ret, frame = cap.read()

    if not ret:
        print('Video end.')
        break

    # save images
    if img_num % frame_interval == 0:
        img_name = os.path.join(IMAGE_PATH, str(uuid.uuid1())+'.jpg')
        cv2.imwrite(img_name, frame)
        print(f'Image {img_num // frame_interval + 1} captured')

    img_num += 1

    if img_num >= num_frames or img_num // frame_interval >= num_imgs_per_video:
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
