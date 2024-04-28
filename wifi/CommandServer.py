## uncomment the following 3 line if you are using Windows
import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath
#########################################################

import sys
sys.path.append('..\AirHokey-Robot')

import cv2
import socket
import threading
from vision.detector import Detector
from motion.robot import robot
from motion.kinematics import *

class CommandServer:
    def __init__(self, 
                 bind_ip="0.0.0.0", 
                 bind_port=59630,
                 client_name1 = "ESP32_A",
                 client_name2 = "ESP32_B"
                 ):

        self.bind_ip = bind_ip
        self.bind_port = bind_port
        self.client_name1 = client_name1
        self.client_name2 = client_name2
        self.targets = {f"{client_name1}": "0\n", 
                        f"{client_name2}": "0\n"}

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.bind_ip, self.bind_port))
        self.server.listen(5)

        self.server.settimeout(10)
        print(f"[*] Listening on {self.bind_ip}:{self.bind_port}")

    def handle_client_connection(self, 
                                 client_socket,
                                 target_point):
        try:
            identifier = client_socket.recv(1024).decode().strip()
            print(f"[*]Received identifier: {identifier}")

            if identifier in self.targets:
                # self.generate_target_command()
                theta1, theta2 = target_point
                self.targets = {
                f"{self.client_name1}": f"{theta1}\n",
                f"{self.client_name2}": f"{theta2}\n"
                }

                target_value = self.targets[identifier]
                
                print(f"[*]Sending target value: {target_value} to {identifier}")
                client_socket.send(target_value.encode("utf-8"))
            else:
                print("[!]Invalid identifier")
        
        finally:
            client_socket.close()









    def run(self):
        while True:
            try:
                print("[*] Waiting for connection...")
                client_sock, address = self.server.accept()
                print(f"[*] Accepted connection from {address[0]}:{address[1]}")



                target_point = (100, 100)
                client_handler = threading.Thread(target=self.handle_client_connection, args=(client_sock,target_point,))
                client_handler.start()
            except Exception as e:
                print(f"[!] Server error: {str(e)}")

if __name__ == "__main__":
    server = CommandServer()
    server.run()
