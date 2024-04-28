import threading
import queue
import cv2
import numpy as np

from vision.detector import Detector
from motion.robot import robot
from motion.control import Controller
from motion.kinematics import *
from wifi.CommandServer import CommandServer


def draw_circle_with_inverse_kinematics(robot, center_x, center_y, radius, num_points=100):
    # Calculate the circle points
    angles = np.linspace(0, 2 * np.pi, num_points)
    circle_x = center_x + radius * np.cos(angles)
    circle_y = center_y + radius * np.sin(angles)

    theta1_values = np.zeros(num_points)
    theta2_values = np.zeros(num_points)

    # Calculate the angles for each point on the circle using inverse kinematics
    for i in range(num_points):
        theta1, theta2 = inverse_kinematics(circle_x[i], circle_y[i], robot)
        theta1_values[i] = theta1
        theta2_values[i] = theta2
        queue.put((theta1_values[i], theta2_values[i]))

    return queue


def server_thread(cm_server,queue):
    while True:
        try:
            print("[*] Waiting for connection...")
            client_sock, address = cm_server.server.accept()
            print(f"[*] Accepted connection from {address[0]}:{address[1]}")

            target_point = queue.get()
            target_point = (target_point[0], target_point[1])
            print(f"Retrieved theta1: {target_point[0]}, theta2: {target_point[1]}")
            client_handler = threading.Thread(target=cm_server.handle_client_connection, args=(client_sock,target_point,))
            client_handler.start()
        except Exception as e:
            print(f"Exception: {e}")
    


def input_thread(queue):
    print("Input thread started. Type 'exit' to stop.")
    while True:
        # Read input from the terminal
        input_data = input("Enter data to send: ")
        if input_data.lower() == 'exit':
            print("Exiting input thread.")
            break
        # Convert the input to a float tuple (assuming the data should be two floats)
        try:
            theta1, theta2 = map(float, input_data.split())
            queue.put((theta1, theta2))
        except ValueError:
            print("Invalid input. Please enter two floats separated by a space.")



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
    input_thread = threading.Thread(target=input_thread, args=(theta_queue,))
    # video_thread = threading.Thread(target=video_processing_and_control, args=(cap, detector, controller, theta_queue, tabel, tabel[0], tabel[2], 400))
    # print_thread = threading.Thread(target=read_and_print_theta, args=(theta_queue,))
    server_thread = threading.Thread(target=server_thread, args=(server,theta_queue))

    # Start threads
    # video_thread.start()
    input_thread.start()
    server_thread.start()
    # print_thread.start()

    # Wait for the video thread to finish
    # video_thread.join()

    # Stop the print thread by sending a stop signal
    # theta_queue.put(np.array([False, False]))
    # print_thread.join()
