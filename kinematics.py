from robot import robot
import numpy as np

def forward_kinematics(theta1,theta2,robot):
    a1 = robot.a1
    a2 = robot.a2
    b1 = robot.b1
    b2 = robot.b2
    w = robot.w
    
    x_R1 = a1 * np.cos(theta1)
    y_R1 = a1 * np.sin(theta1)
    x_R2 = a2 * np.cos(theta2) + w
    y_R2 = a2 * np.sin(theta2)

    v1 = (y_R1 - y_R2) / (x_R2 - x_R1)
    v2 = (b1**2 - b2**2 - a1**2 + x_R2**2 + y_R2**2) / (2 * (x_R2 - x_R1))
    v3 = 1 + v1**2
    v4 = 2 * (v1*v2 - v1*x_R1 - y_R1)
    v5 = (v2-x_R1)**2+y_R1**2-b1**2
    
    discriminant = np.sqrt(v4**2 - 4*v3*v5)#mode1
    target_y = (-v4 + discriminant) / (2*v3)
    target_x = v1 * target_y + v2

    return target_x, target_y

def inverse_kinematics(target_x,target_y,robot):
    a1 = robot.a1
    a2 = robot.a2
    b1 = robot.b1
    b2 = robot.b2
    w = robot.w
    working_mode = robot.working_mode

    c1 = np.sqrt(target_x**2 + target_y**2)
    c2 = np.sqrt((target_x - w)**2 + target_y**2)

    alpha1 = np.arccos((c2**2 - c1**2 - w**2) / (-2 * c1 * w))
    alpha2 = np.arccos((-target_x + w) / c2)

    beta1 = np.arccos((b1**2 - a1**2 - c1**2) / (-2 * a1 * c1))
    beta2 = np.arccos((b2**2 - a2**2 - c2**2) / (-2 * a2 * c2))

    if working_mode == '++':
        theta1 = alpha1 + beta1
        theta2 = np.pi-(alpha2 + beta2)
    elif working_mode == '+-':
        theta1 = alpha1 + beta1
        theta2 = np.pi-(alpha2 - beta2)
    elif working_mode == '-+':
        theta1 = alpha1 - beta1
        theta2 = np.pi-(alpha2 + beta2)
    elif working_mode == '--':
        theta1 = alpha1 - beta1
        theta2 = np.pi-(alpha2 - beta2)

    return theta1, theta2
