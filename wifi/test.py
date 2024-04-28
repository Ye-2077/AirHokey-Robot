import numpy as np
import matplotlib.pyplot as plt

def CubicPolyInterpolation(state_initial: np.array,
                           state_end: np.array,
                           delta_t: float):
    """
    Cubic polynomial interpolation
    - state_initial/end: [theta1, d_theta1, theta2, d_theta2]
    - delta_t: cost time
    """
    
    x = np.array([0,delta_t])
    f = np.array([x**3, x**2, x, np.ones_like(x)]).T
    df = np.array([3*x**2, 2*x, np.ones_like(x), np.zeros_like(x)]).T

    A = np.vstack([f, df])
    b = np.array([state_initial[0], state_end[0],
                  state_initial[1], state_end[1]])
    coefficients = np.linalg.solve(A, b)

    return coefficients



if __name__ == "__main__":
    state_initial = np.array([0, 0, 1, 1])
    state_end = np.array([2, 0, 1, 2])
    delta_t = 1
    coefficients = CubicPolyInterpolation(state_initial, state_end, delta_t)
    print(coefficients)