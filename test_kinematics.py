from motion.kinematics import forward_kinematics, inverse_kinematics
from motion.robot import robot
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def plot_mechanism(robot, theta1, theta2):
    fig, ax = plt.subplots()
    
    # Calculate end-effector position
    x_end, y_end = forward_kinematics(theta1, theta2, robot)
    
    # Positions of the fixed pivots (assuming they are at (0,0) and (w,0))
    x_fixed1, y_fixed1 = 0, 0
    x_fixed2, y_fixed2 = robot.w, 0
    
    # Calculate positions of the moving pivots (end of the driving links)
    x_input1, y_input1 = robot.a1 * np.cos(theta1), robot.a1 * np.sin(theta1)
    x_input2, y_input2 = robot.a2 * np.cos(theta2) + robot.w, robot.a2 * np.sin(theta2)
    
    # Plot the driving links
    ax.plot([x_fixed1, x_input1], [y_fixed1, y_input1], 'ro-')  # Link 1
    ax.plot([x_fixed2, x_input2], [y_fixed2, y_input2], 'ro-')  # Link 2
    
    # Plot the coupler (This is a simplification; exact calculation depends on the specific five-bar linkage)
    ax.plot([x_input1, x_end], [y_input1, y_end], 'bo-')  # Link 3 (Coupler)
    ax.plot([x_input2, x_end], [y_input2, y_end], 'bo-')  # Link 4 (Coupler)
    
    # Plot the fixed base
    ax.plot([x_fixed1, x_fixed2], [y_fixed1, y_fixed2], 'ko-')  # Base Link
    
    # Setting plot limits and aspect ratio
    plot_buffer = 35
    ax.set_xlim(-robot.a1 - plot_buffer, robot.w + robot.a2 + plot_buffer)
    ax.set_ylim(-max(robot.a1, robot.a2) - plot_buffer, max(robot.a1, robot.a2) + plot_buffer)
    ax.set_aspect('equal')
    plt.show()

#define robot configuration (should be adjusted according to the real printed parts)
robot = robot(23.5,23.5,33,33,25,working_mode='++')

def animate_mechanism(robot, frames=100, interval=100):
    fig, ax = plt.subplots()
    line, = plt.plot([], [], 'ro-')
    plt.xlim(-robot.a1 - 10, robot.w + robot.a2 + 10)
    plt.ylim(-max(robot.a1, robot.a2) - 10, max(robot.a1, robot.a2) + 10)
    
    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        # Independently vary theta1 and theta2
        theta1 = np.radians(frame)
        theta2 = np.radians(2 * frame % 360)  # To ensure theta2 varies differently
        x_end, y_end = forward_kinematics(theta1, theta2, robot)
        
        # Plot everything as before
        x_input1, y_input1 = robot.a1 * np.cos(theta1), robot.a1 * np.sin(theta1)
        x_input2, y_input2 = robot.a2 * np.cos(theta2) + robot.w, robot.a2 * np.sin(theta2)
        
        # Update the line data for the animation
        line.set_data([0, x_input1, x_end, x_input2, robot.w, 0], 
                      [0, y_input1, y_end, y_input2, 0, 0])
        return line,

    ani = FuncAnimation(fig, update, frames=np.linspace(0, 360, frames), init_func=init, interval=interval, blit=True)
    plt.show()

# Example usage of the animation with non-matching theta1 and theta2

# plot_mechanism(robot, np.radians(90), np.radians(90))

# animate_mechanism(robot)

def draw_circle_with_inverse_kinematics_visualize(robot, center_x, center_y, radius, num_points=100):
    # Calculate the circle points
    angles = np.linspace(0, 2 * np.pi, num_points)
    circle_x = center_x + radius * np.cos(angles)
    circle_y = center_y + radius * np.sin(angles)

    fig, ax = plt.subplots()
    ax.set_xlim(-robot.a1 - robot.a2, robot.w + robot.a1 + robot.a2)
    ax.set_ylim(-robot.a1 - robot.a2, robot.a1 + robot.a2)
    ax.set_aspect('equal')
    line, = ax.plot([], [], 'bo-', lw=2)
    path, = ax.plot([], [], 'g-', lw=2)

    # Initialize animation
    def init():
        line.set_data([], [])
        path.set_data([], [])
        return line, path

    # Update function for animation
    def update(i):
        # Compute inverse kinematics for the current circle point
        target_x, target_y = circle_x[i], circle_y[i]
        theta1, theta2 = inverse_kinematics(target_x, target_y, robot)
        
        # Compute forward kinematics for the angles to get the mechanism configuration
        fk_x, fk_y = forward_kinematics(theta1, theta2, robot)
        
        # Update the line data
        input1_x, input1_y = robot.a1 * np.cos(theta1), robot.a1 * np.sin(theta1)
        input2_x, input2_y = robot.a2 * np.cos(theta2) + robot.w, robot.a2 * np.sin(theta2)
        
        mechanism_points_x = [0, input1_x, fk_x, input2_x, robot.w]
        mechanism_points_y = [0, input1_y, fk_y, input2_y, 0]

        line.set_data(mechanism_points_x, mechanism_points_y)
        
        # Draw the path
        if i == 0:
            path.set_data(mechanism_points_x[2], mechanism_points_y[2])
        else:
            prev_data_x, prev_data_y = path.get_data()
            new_data_x = np.append(prev_data_x, mechanism_points_x[2])
            new_data_y = np.append(prev_data_y, mechanism_points_y[2])
            path.set_data(new_data_x, new_data_y)

        return line, path

    ani = FuncAnimation(fig, update, frames=range(num_points),
                        init_func=init, blit=True, repeat=False)

    plt.show()

# Example usage:
center_x = 20  # x-coordinate of the circle center
center_y = 20  # y-coordinate of the circle center
radius = 5    # Radius of the circle

# draw_circle_with_inverse_kinematics_visualize(robot, center_x, center_y, radius)
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

    return theta1_values, theta2_values


def draw_line_with_inverse_kinematics(robot, start_x, start_y, end_x, end_y, num_points=100):
    # Generate the target points along the line
    line_x = np.linspace(start_x, end_x, num_points)
    line_y = np.linspace(start_y, end_y, num_points)
    
    fig, ax = plt.subplots()
    ax.set_xlim(-robot.a1 - robot.a2, robot.w + robot.a1 + robot.a2)
    ax.set_ylim(-robot.a1 - robot.a2, robot.a1 + robot.a2)
    ax.set_aspect('equal')
    mechanism_line, = ax.plot([], [], 'ro-', lw=2)
    path_line, = ax.plot([], [], 'g-', lw=2)

    # Initialize the animation
    def init():
        mechanism_line.set_data([], [])
        path_line.set_data([], [])
        return mechanism_line, path_line

    # Update function for the animation
    def update(i):
        # Get the current target point
        target_x, target_y = line_x[i], line_y[i]
        
        # Calculate the joint angles using inverse kinematics
        theta1, theta2 = inverse_kinematics(target_x, target_y, robot)
        
        # Plot the mechanism for the current joint angles
        x_input1, y_input1 = robot.a1 * np.cos(theta1), robot.a1 * np.sin(theta1)
        x_input2, y_input2 = robot.a2 * np.cos(theta2) + robot.w, robot.a2 * np.sin(theta2)
        
        # Get the end effector position from forward kinematics (not provided here)
        # x_end, y_end = forward_kinematics(theta1, theta2, robot)
        # For the purpose of this visualization, we'll use the target points
        x_end, y_end = target_x, target_y
        
        mechanism_line.set_data([0, x_input1, x_end, x_input2, robot.w, 0],
                                [0, y_input1, y_end, y_input2, 0, 0])

        # Update the path line
        if i == 0:
            path_line.set_data(x_end, y_end)
        else:
            prev_data_x, prev_data_y = path_line.get_data()
            new_data_x = np.append(prev_data_x, x_end)
            new_data_y = np.append(prev_data_y, y_end)
            path_line.set_data(new_data_x, new_data_y)
        
        return mechanism_line, path_line
    
def draw_line_with_inverse_kinematics_visualize(robot, start_x, start_y, end_x, end_y, num_points=100):
    # Generate the target points along the line
    line_x = np.linspace(start_x, end_x, num_points)
    line_y = np.linspace(start_y, end_y, num_points)
    
    fig, ax = plt.subplots()
    ax.set_xlim(-robot.a1 - robot.a2, robot.w + robot.a1 + robot.a2)
    ax.set_ylim(-robot.a1 - robot.a2, robot.a1 + robot.a2)
    ax.set_aspect('equal')
    mechanism_line, = ax.plot([], [], 'ro-', lw=2)
    path_line, = ax.plot([], [], 'g-', lw=2)

    # Initialize the animation
    def init():
        mechanism_line.set_data([], [])
        path_line.set_data([], [])
        return mechanism_line, path_line

    # Update function for the animation
    def update(i):
        # Get the current target point
        target_x, target_y = line_x[i], line_y[i]
        
        # Calculate the joint angles using inverse kinematics
        theta1, theta2 = inverse_kinematics(target_x, target_y, robot)
        
        # Plot the mechanism for the current joint angles
        x_input1, y_input1 = robot.a1 * np.cos(theta1), robot.a1 * np.sin(theta1)
        x_input2, y_input2 = robot.a2 * np.cos(theta2) + robot.w, robot.a2 * np.sin(theta2)
        
        # Get the end effector position from forward kinematics (not provided here)
        # x_end, y_end = forward_kinematics(theta1, theta2, robot)
        # For the purpose of this visualization, we'll use the target points
        x_end, y_end = target_x, target_y
        
        mechanism_line.set_data([0, x_input1, x_end, x_input2, robot.w, 0],
                                [0, y_input1, y_end, y_input2, 0, 0])

        # Update the path line
        if i == 0:
            path_line.set_data(x_end, y_end)
        else:
            prev_data_x, prev_data_y = path_line.get_data()
            new_data_x = np.append(prev_data_x, x_end)
            new_data_y = np.append(prev_data_y, y_end)
            path_line.set_data(new_data_x, new_data_y)
        
        return mechanism_line, path_line

    ani = FuncAnimation(fig, update, frames=num_points, init_func=init, blit=True, repeat=False)
    plt.show()

# Example usage:
# start_point = (-25,25)  # Start point of the line (x, y)
# end_point = (50,25)   # End point of the line (x, y)
start_point = (-25,20)  # Start point of the line (x, y)
end_point = (50,20)   # End point of the line (x, y)
num_points = 100        # Number of points to interpolate along the line
draw_line_with_inverse_kinematics_visualize(robot, *start_point, *end_point, num_points)