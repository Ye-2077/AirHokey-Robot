import threading
import queue
import cv2
import numpy as np
import pygame

from vision.detector import Detector
from motion.robot import robot
from motion.control import Controller
from wifi.CommandServer import CommandServer
from motion.visualization import VisualSim
from motion.kinematics import *
from motion.robot import robot as Robot

def video_processing_and_control(cap, 
                                 detector, 
                                 controller, 
                                 queue, 
                                 tabel, 
                                 table_left, 
                                 table_top, 
                                 ready_line):
    
    print("Starting video thread")

    cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('YOLO', 800, 500)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        frame, trajectory = detector.detect(frame, render_trajectory=True)
        frame, future_trajectory = detector.predict_trajectory(frame, render_trajectory=True)

        # Generate target
        target_position = future_trajectory[-1] - np.array([table_left, table_top])
        theta1, theta2 = controller.generate_target_command(target_position)
       
        if theta1 is None or theta2 is None:
            theta1, theta2 = controller.generate_target_command(np.array([target_position[0], ready_line]))

        # Store the angles in queue
        queue.put((theta1, theta2))

        cv2.imshow('YOLO', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

def read_and_print_theta(queue):
    print("Starting print thread")
    while True:
        try:
            theta1, theta2 = queue.get(timeout=1)
            print(f"Retrieved theta1: {theta1}, theta2: {theta2}")
        except Exception as e:
            print(f"Exception: {e}")
        except queue.Empty:
            print("Queue is empty, waiting for data")
            continue

        # theta1, theta2 = queue.get()
        # print(f"Retrieved theta1: {theta1}, theta2: {theta2}")


def server_thread(cm_server,queue):
    while True:
        try:
            print("[*] Waiting for connection...")
            client_sock, address = cm_server.server.accept()
            print(f"[*] Accepted connection from {address[0]}:{address[1]}")

            target_point = queue.get()
            print(f"Retrieved theta1: {target_point[0]}, theta2: {target_point[1]}")
            client_handler = threading.Thread(target=cm_server.handle_client_connection, args=(client_sock,target_point,))
            client_handler.start()
        except Exception as e:
            print(f"Exception: {e}")
    

def sim_thread(theta_queue):
    sim = VisualSim(Robot(a1=400, a2=400, b1=500, b2=500, w=500))
    running = True
    theta1 = np.pi / 2
    theta2 = np.pi / 2
    while running:
        mouse_x, mouse_y = pygame.mouse.get_pos()  # Get current mouse position
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    
        theta1, theta2 = theta_queue.get()
        if theta1 is None or theta2 is None:
            continue
        target_x, target_y = forward_kinematics(theta1, theta2, Robot(a1=400, a2=400, b1=500, b2=500, w=500, working_mode='++'))
        target_y = 200

        sim.visualize(theta1, theta2, mouse_x, mouse_y, target_x, target_y)
        pygame.time.wait(50)  # Delay to slow down the animation

    pygame.quit()




if __name__ == '__main__':
    bind_ip="0.0.0.0"
    bind_port=59630
    server = CommandServer(client_name1="ESP32_A", 
                           client_name2="ESP32_B", 
                           bind_ip=bind_ip, 
                           bind_port=bind_port)


    robot = robot(a1=400, a2=400, b1=500, b2=500, w=500, working_mode='++')
    controller = Controller(robot=robot)

    cap = cv2.VideoCapture('vision/samples/sample1.mp4')
    tabel = (250, 1600, 140, 930)
    detector = Detector('vision/models/yolov5_1.pt', tabel=tabel, device='cuda')
    
    # Initialize the Kalman filter
    init_puck_x, init_puck_y = detector.detect_pos(detector.model(cap.read()[1]), target='puck')[:2]
    detector.kalman = detector.init_kalman(init_puck_x, init_puck_y)

    theta_queue = queue.Queue()

    # Create threads
    video_thread = threading.Thread(target=video_processing_and_control, args=(cap, detector, controller, theta_queue, tabel, tabel[0], tabel[2], 400))
    print_thread = threading.Thread(target=read_and_print_theta, args=(theta_queue,))
    server_thread = threading.Thread(target=server_thread, args=(server,theta_queue))
    sim_thread = threading.Thread(target=sim_thread, args=(theta_queue,))

    # Start threads
    video_thread.start()
    sim_thread.start()
    server_thread.start()
    # print_thread.start()

    # Wait for the video thread to finish
    # video_thread.join()

    # Stop the print thread by sending a stop signal
    # theta_queue.put(np.array([False, False]))
    # print_thread.join()
