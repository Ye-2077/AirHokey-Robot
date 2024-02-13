import numpy as np
import matplotlib.pyplot as plt

def CubicPolyInterpolation(state_initial: np.array,
                           state_end: np.array,
                           delta_t: float,
                           plot=True):
    """
    Cubic polynomial interpolation
    - state_initial/end: [theta, d_theta]
    - delta_t: cost time
    """
    
    x = np.array([0,delta_t])
    f = np.array([x**3, x**2, x, np.ones_like(x)]).T
    df = np.array([3*x**2, 2*x, np.ones_like(x), np.zeros_like(x)]).T

    A = np.vstack([f, df])
    b = np.array([state_initial[0], state_end[0],
                  state_initial[1], state_end[1]])
    coefficients = np.linalg.solve(A, b)

    if plot:
        time = np.linspace(0, delta_t, 100)
        trajactory = np.array([time**3, time**2, time, np.ones_like(time)]).T @ coefficients
        velocity = np.array([3*time**2, 2*time, np.ones_like(time), np.zeros_like(time)]).T @ coefficients
        acceleration = np.array([6*time, 2*np.ones_like(time), np.zeros_like(time), np.zeros_like(time)]).T @ coefficients
        plt.plot(time, trajactory, label='position')
        plt.plot(time, velocity, label='velocity')
        plt.plot(time, acceleration, label='acceleration')
        plt.legend()
        plt.grid()
        plt.show()

    return coefficients


if __name__ == "__main__":
    state_initial = np.array([0, 0])
    state_end = np.array([2, -1])
    delta_t = 100
    coefficients = CubicPolyInterpolation(state_initial, state_end, delta_t)
    print(coefficients)