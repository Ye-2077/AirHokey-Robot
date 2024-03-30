import cv2
import time
# 打开默认相机
cap = cv2.VideoCapture(0)
# 初始化帧计数器
frame_count = 0

# 初始化时间戳
start_time = time.time()
# 获取帧宽度和高度
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print("原始宽度:", width)
print("原始高度:", height)

# 设置帧宽度和高度
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 再次获取设置后的帧宽度和高度
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print("设置后的宽度:", width)
print("设置后的高度:", height)
try:
    while True:
        # 读取一帧
        ret, frame = cap.read()
        
        # 如果正确读取帧，ret为True
        if not ret:
            break
        
        frame_count += 1
        
        # 显示帧
        cv2.imshow('Frame', frame)
        
        # 按 'q' 退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 计算总时间和平均帧率
    total_time = time.time() - start_time
    average_fps = frame_count / total_time
    
    print(f"总帧数: {frame_count}, 总时间: {total_time:.2f}秒, 平均帧率: {average_fps:.2f} FPS")

# 当一切完成后，释放捕获器
cap.release()
cv2.destroyAllWindows()
