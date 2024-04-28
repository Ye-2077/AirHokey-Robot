import numpy as np
import matplotlib.pyplot as plt
from motion.robot import robot
from motion.kinematics import forward_kinematics

# Initialize the robot with some example parameters
my_robot = robot(23.5,23.5,33,33,25, working_mode='-+')

# Generate a grid of theta1 and theta2 values
theta1_values = np.linspace(-np.pi, np.pi, 100)
theta2_values = np.linspace(-np.pi, np.pi, 100)
theta1_grid, theta2_grid = np.meshgrid(theta1_values, theta2_values)

# Compute the workspace
x_vals = []
y_vals = []
for theta1, theta2 in zip(np.ravel(theta1_grid), np.ravel(theta2_grid)):
    x, y = forward_kinematics(theta1, theta2, my_robot)
    x_vals.append(x)
    y_vals.append(y)

# Plotting
plt.figure(figsize=(8, 8))
plt.scatter(x_vals, y_vals, color='blue', s=1)
plt.title('Robot Workspace')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True)
plt.axis('equal')
plt.show()
# import numpy as np
# import matplotlib.pyplot as plt
# from robot import robot

# # ... (Rest of your code, including robot initialization and the modified forward_kinematics function)

# def plot_workspace(robot):
#     # Generate a grid of theta1 and theta2 values within their physical limits.
#     theta1_values = np.linspace(-np.pi, np.pi, 200)
#     theta2_values = np.linspace(-np.pi, np.pi, 200)

#     x_vals = []
#     y_vals = []
#     for theta1 in theta1_values:
#         for theta2 in theta2_values:
#             x, y = forward_kinematics(theta1, theta2, robot)
#             # Only keep the points above the line connecting the joints
#             if y >= (robot.a1 * np.sin(theta1)):  # This condition ensures the target is above the joint line
#                 x_vals.append(x)
#                 y_vals.append(y)

#     # Plotting the workspace
#     plt.figure(figsize=(10, 10))
#     plt.scatter(x_vals, y_vals, color='blue', s=1)
#     plt.title('Robot Workspace without -solutions')
#     plt.xlabel('X Position')
#     plt.ylabel('Y Position')
#     plt.grid(True)
#     plt.axis('equal')
#     plt.show()

# # Assuming 'my_robot' is an instance of 'robot' class with proper parameters set.
# plot_workspace(my_robot)
