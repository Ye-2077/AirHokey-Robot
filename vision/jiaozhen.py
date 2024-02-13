import cv2
import numpy as np

# 读取视频文件或摄像头
cap = cv2.VideoCapture("vision/samples/output_video (online-video-cutter.com).mp4")  # 替换为你的视频文件路径或摄像头索引

# 获取视频的宽度和高度
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# 设置图像窗口大小
cv2.namedWindow('Corrected', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Corrected', 800, 600)

# 设置鱼眼矫正参数
K = np.array([[width, 0.5, (width-1)/2],
              [0, height, (height-1)/2],
              [0, 0, 1]], dtype=np.float32)

D = np.zeros((4, 1), dtype=np.float32)
D = np.array([0, 0.001, 0, 0.001], dtype=np.float32)

# 设置输出视频文件
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output_video2.mp4', fourcc, 20.0, (width, height))

while True:
    # 读取视频帧
    ret, frame = cap.read()
    if not ret:
        break

    # 进行鱼眼矫正
    corrected_frame = cv2.fisheye.undistortImage(frame, K, D=D, Knew=K)

    # 显示矫正后的图像
    cv2.imshow('Corrected', corrected_frame)

    # 写入矫正后的帧到输出视频文件
    out.write(corrected_frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
out.release()
cv2.destroyAllWindows()
