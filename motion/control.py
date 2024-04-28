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

class Controller:
    def __init__(self, 
                 robot
                #  clinent_name1="ESP32_A",
                #  clinent_name2="ESP32_B"
                 ):
        self.robot = robot
        # self.clinent_name1 = clinent_name1
        # self.clinent_name2 = clinent_name2
        # self.targets = {f"{clinent_name1}": "-2\n", 
        #                 f"{clinent_name2}": "0\n"}

    def generate_target_command(self, target_point):
        target_x = target_point[0]
        target_y = target_point[1]
        theta1, theta2 = inverse_kinematics(target_x, target_y, self.robot)

        # self.targets = {
        #     f"{self.clinent_name1}": f"{theta1}\n",
        #     f"{self.clinent_name2}": f"{theta2}\n"
        # }
        return theta1, theta2