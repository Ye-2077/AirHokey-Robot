import threading
import queue
import cv2
import numpy as np
import socket
import time
import math
import pygame

from vision.detector import Detector
from motion.robot import robot
from motion.control import Controller
from wifi.CommandServer import CommandServer
from motion.kinematics import inverse_kinematics
from motion.visualization import VisualSim

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


def server_thread(cm_server, theta_queue):
    client_connections = []  # List to store (client_socket, address, identifier) tuples

    def accept_connections():
        while True:
            try:
                client_sock, address = cm_server.server.accept()
                identifier = client_sock.recv(1024).decode().strip()  # 假设客户端连接后立即发送其标识符
                client_connections.append((client_sock, address, identifier))
                print(f"[*] Accepted connection from {identifier} at {address[0]}:{address[1]}")
            except socket.error as e:
                print(f"Socket error during accept: {e}")
                break

    # Thread for accepting connections
    accept_thread = threading.Thread(target=accept_connections)
    accept_thread.daemon = True
    accept_thread.start()

    while True:
        try:
            theta1, theta2 = theta_queue.get()  # Assume theta1 for ESP32_A, theta2 for ESP32_B
            for client_sock, address, identifier in client_connections:
                if identifier == 'ESP32_A':
                    message = f"{theta1}\n"
                elif identifier == 'ESP32_B':
                    message = f"{theta2}\n"
                else:
                    continue  # Skip if identifier is unknown

                try:
                    client_sock.sendall(message.encode('utf-8'))
                    print(f"Sent {message.strip()} to {identifier} at {address[0]}:{address[1]}")
                    
                except socket.error:
                    client_connections.remove((client_sock, address, identifier))
                    client_sock.close()
                    print(f"[*] Lost connection to {identifier} at {address[0]}:{address[1]}")
        except Exception as e:
            print(f"Exception in broadcasting: {e}")

    
def input_and_enqueue_angles(theta_queue):
    print("Start reading angles from terminal. Type 'exit' to stop.")
    while True:
        user_input = input("Enter theta1, theta2 (comma-separated): ")
        if user_input.lower() == 'exit':
            break
        try:
            theta1, theta2 = map(float, user_input.split(','))
            theta_queue.put((theta1, theta2))
        except ValueError:
            print("Invalid input. Please enter two comma-separated numbers.")
        except Exception as e:
            print(f"Error: {e}")

def input_target_position(queue):
    print("Start reading target position from terminal. Type 'exit' to stop.")
    while True:
        user_input = input("Enter target position (x, y) (comma-separated): ")
        if user_input.lower() == 'exit':
            break
        try:
            x, y = map(float, user_input.split(','))
            myrobot = robot(a1 = 0.2, a2 = 0.2, b1 = 0.25, b2 = 0.25, w = 0.25)
            theta1, theta2 = inverse_kinematics(x, y, myrobot)
            theta1 = theta1 - math.pi/2
            theta2 = -(theta2 - math.pi/2)
            queue.put((theta1, theta2))
        except ValueError:
            print("Invalid input. Please enter two comma-separated numbers.")
        except Exception as e:
            print(f"Error: {e}")


def oscillating_angles_thread(theta_queue, period=4, amplitude=1.57, threshold=0.5):
    print("Starting oscillating angles thread")
    start_time = time.time()
    last_theta = 0  # Initialize last_theta to compare with new angles

    while True:
        elapsed = time.time() - start_time
        current_theta = amplitude * math.sin((2 * math.pi / period) * elapsed)  # Current angle

        # Check if the change in angle is greater than the threshold
        if abs(current_theta - last_theta) > threshold:
            # Calculate intermediate values using cubic interpolation
            for t in np.linspace(0, 1, num=4):  # More steps can be used for smoother interpolation
                interpolated_theta = (1 - t) ** 3 * last_theta + 3 * t * (1 - t) ** 2 * (last_theta + (current_theta - last_theta) / 3) + \
                                     3 * t ** 2 * (1 - t) * (last_theta + 2 * (current_theta - last_theta) / 3) + t ** 3 * current_theta
                theta_queue.put((interpolated_theta, -interpolated_theta))
                time.sleep(0.1)  # Time between interpolation steps
        else:
            # If change is below threshold, just put the current angle
            theta_queue.put((current_theta, -current_theta))

        last_theta = current_theta  # Update last_theta
        time.sleep(0.1)  # Sleep at the end of the loop to control the update frequency


def sim_control_thread(theta_queue):
    sim = VisualSim(robot(a1=200, a2=200, b1=250, b2=250, w=250))
    running = True
    theta1 = np.pi / 2
    theta2 = np.pi / 2
    while running:
        mouse_x, mouse_y = pygame.mouse.get_pos()  # Get current mouse position
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Calculate inverse kinematics
        if mouse_x < 130:
            target_x = -130
        elif mouse_x > 370:
            target_x = 370
        else:
            target_x = (mouse_x-sim.center_x)*2

        # print("target_x: ", target_x)
        # print("mouse_x: ", mouse_x)

        target_y = 220

        # try:
        #     target_y = (mouse_y-sim.center_y)*2
        #     theta1, theta2 = inverse_kinematics(target_x, target_y, sim.robot)
        #     if theta1 is not None and theta2 is not None:
        #         theta_queue.put((theta1-np.pi/2, np.pi-theta2-np.pi/2))
        # except Exception as e:
        #     print(f"Error: {e}")
        #     continue

        theta1, theta2 = inverse_kinematics(target_x, target_y, sim.robot)
        theta_queue.put((theta1-np.pi/2, np.pi-theta2-np.pi/2))


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

    theta_queue = queue.Queue()

    # Create threads
    input_thread = threading.Thread(target=input_and_enqueue_angles, args=(theta_queue,))
    # oscillating_thread = threading.Thread(target=oscillating_angles_thread, args=(theta_queue,))
    # smooth_oscillating_thread = threading.Thread(target=smooth_oscillating_angles_thread, args=(theta_queue,))
    sim_thread = threading.Thread(target=sim_control_thread, args=(theta_queue,))
    server_thread = threading.Thread(target=server_thread, args=(server,theta_queue))

    read_and_print_theta = threading.Thread(target=read_and_print_theta, args=(theta_queue,))

    # Start threads
    server_thread.start()
    sim_thread.start()
    read_and_print_theta.start()
    input_thread.start()
    # oscillating_thread.start()
    # smooth_oscillating_thread.start()

