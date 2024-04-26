import sys
sys.path.append('..\AirHokey-Robot')

import pygame
import numpy as np

from motion.robot import robot
from motion.kinematics import *

class VisualSim:
    def __init__(self, robot):
        self.robot = robot
        pygame.init()
        self.width, self.height = 500, 500
        self.screen = pygame.display.set_mode((self.width, self.height))

        # Define colors
        self.background_color = (0, 0, 0)
        self.link_color = (255, 0, 0)
        self.joint_color = (0, 255, 0)
        self.end_effector_color = (255, 255, 255)  # White for end effector
        self.base_color_1 = (0, 0, 255)  # Blue for base points
        self.base_color_2 = (0, 255, 255)  # Cyan for other base points

        # Scale to improve visualization
        self.scale = 1/2
        self.center_x = self.width // 2 - 50
        self.center_y = self.height // 2 - 50

        # Font setup
        self.font = pygame.font.Font(None, 24)

    def visualize(self, theta1, theta2, mouse_x, mouse_y, target_x, target_y):
        # Clear the screen at the start of each draw call
        self.screen.fill(self.background_color)

        end_X, end_Y = forward_kinematics(theta1, theta2, self.robot)
        joint1_X = self.robot.a1 * np.cos(theta1)
        joint1_Y = self.robot.a1 * np.sin(theta1)
        joint2_X = self.robot.a2 * np.cos(theta2) + self.robot.w
        joint2_Y = self.robot.a2 * np.sin(theta2)

        points = [
            (self.center_x, self.center_y),  # Origin
            (self.center_x + int(joint1_X * self.scale), self.center_y + int(joint1_Y * self.scale)),
            (self.center_x + int(end_X * self.scale), self.center_y + int(end_Y * self.scale)),
            (self.center_x + int(joint2_X * self.scale), self.center_y + int(joint2_Y * self.scale)),
            (self.center_x + int(self.robot.w * self.scale), self.center_y)  # Adjusted for w only
        ]

        # Draw the links and joints
        pygame.draw.lines(self.screen, self.link_color, False, points, 2)
        
        # Draw circles for origin and base end in specific colors, and end effector in white
        pygame.draw.circle(self.screen, self.base_color_1, points[0], 5)  # Origin in blue
        pygame.draw.circle(self.screen, self.base_color_2, points[4], 5)  # Base end in cyan
        pygame.draw.circle(self.screen, self.end_effector_color, points[2], 5)  # End effector in white

        # Draw remaining joints in green
        pygame.draw.circle(self.screen, self.joint_color, points[1], 5)
        pygame.draw.circle(self.screen, self.joint_color, points[3], 5)

        # Display mouse coordinates
        text = self.font.render(f"Mouse: {mouse_x}, {mouse_y}", True, (255, 255, 255))
        text_theta1 = self.font.render(f"Theta1: {theta1:.2f}", True, (255, 255, 255))
        text_theta2 = self.font.render(f"Theta2: {theta2:.2f}", True, (255, 255, 255))
        text_target = self.font.render(f"Target: {target_x}, {target_y}", True, (255, 255, 255))

        self.screen.blit(text, (self.width - text.get_width() - 10, self.height - text.get_height() - 10))
        self.screen.blit(text_target, ((self.width - text_target.get_width() - 10, self.height - text_target.get_height() - 40)))

        self.screen.blit(text_theta2, (10, self.height - text_theta1.get_height() - 10))
        self.screen.blit(text_theta1, (10, self.height - text_theta2.get_height() - 40))
        pygame.display.flip()  # Update the display

    def run_simulation(self):
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
                target_x = (mouse_x-self.center_x)*2

            print("target_x: ", target_x)
            print("mouse_x: ", mouse_x)

            target_y = 200
            theta1, theta2 = inverse_kinematics(target_x, target_y, self.robot)




            self.visualize(theta1, theta2, mouse_x, mouse_y, target_x, target_y)

            pygame.time.wait(50)  # Delay to slow down the animation

        pygame.quit()

if __name__ == "__main__":
    # Initialize your robot parameters
    robot = robot(a1=200, a2=200, b1=250, b2=250, w=250)
    sim = VisualSim(robot)
    sim.run_simulation()
