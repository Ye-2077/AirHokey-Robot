import cv2
import numpy as np

# 预设桌子边界颜色在HSV色彩空间中的范围
# 注意：这些值需要你根据实际情况调整
lower_hsv = np.array([0, 0, 168])
upper_hsv = np.array([172, 111, 255])

# 加载视频
cap = cv2.VideoCapture('vision/samples/sample1.mp4')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 转换到HSV色彩空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 创建一个掩膜，只保留指定的颜色范围
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # 可选：对掩膜进行膨胀和腐蚀操作，去除小噪点
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.erode(mask, None, iterations=2)

    # 在原图上应用掩膜，只显示桌子边界
    frame_with_border = cv2.bitwise_and(frame, frame, mask=mask)
    
    # 寻找掩膜中的轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 根据情况可能需要过滤轮廓，例如选择最大的轮廓
    if contours:
        # 可选：选择最大的轮廓画出边界
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(frame_with_border, [largest_contour], -1, (0, 255, 0), 2)

    # 显示结果
    cv2.imshow('YOLO', frame_with_border)
    
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
