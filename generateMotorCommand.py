import numpy as np
def give_motor_command(image_list):
    theta1 = 0
    theta2 = 0
    angle_command = np.array([theta1, theta2])
    return angle_command

def predict_current_state(image_list):
    x_ball = 0
    y_ball = 0
    x_vel_ball = 0
    y_vel_ball = 0
    current_state = np.array([x_ball,y_ball,x_vel_ball,y_vel_ball])
    return current_state


def plan_next_move(current_state):
    target_x = 0
    target_y = 0
    target_point = np.array([target_x, target_y])
    return target_point