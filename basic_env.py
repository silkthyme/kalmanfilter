# Environment Setup
# Objective: Simulate Motion of the Hyperloop Pod
# Key Components: 1) Equation to Simulate true motion of the Hyperloop Pod
#                 2) Equation to model measured Position
#                 3) Physics Based Expected Position and Velocity Equation
# Assumptions: - Sample Measured position with 1m standard deviation
#              - Sample Measured velocity with 0.1km/hr standard deviaation
#              - Constant Accelaration

import filterpy
from filterpy.filterpy.kalman import KalmanFilter
from filterpy.filterpy.stats import plot_covariance_ellipse

# State Transition Function: choose state variables x and y for 2 dimensions
# x_bar = Fx
# transition function is implemented as: next state = matrix F * previous state
tracker = KalmanFilter(dim_x=4, dim_z=2) 
dt = 1. # time step 1 second

tracker.F = np.array([1, dt, 0, 0],
                    [0, 1, 0 ,0],
                    [0, 0, 1, dt],
                    [0, 0, 0, 1])

# Control Function
# tracker.B

# Measurement Function H
tracker.H = np.array([[1/0.3048, 0, 0,        0],
                      [0,        0, 1/0.3048, 0]])

# Measurement Noise Matrix R
tracker.R = np.array([[5., 0],
                      [0, 5]])

# Initial Conditions
tracker.x = np.array([[0, 0, 0, 0]]).T
tracker.P = np.eye(4) * 500.

# Implement the Filter
R_std = 0.35
Q_std = 0.04

def tracker1():
    tracker = KalmanFilter(dim_x=4, dim_z=2)
    dt = 1.0   # time step

    tracker.F = np.array([[1, dt, 0,  0],
                          [0,  1, 0,  0],
                          [0,  0, 1, dt],
                          [0,  0, 0,  1]])
    tracker.u = 0.
    tracker.H = np.array([[1/0.3048, 0, 0, 0],
                          [0, 0, 1/0.3048, 0]])

    tracker.R = np.eye(2) * R_std**2
    q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_std**2)
    tracker.Q = block_diag(q, q)
    tracker.x = np.array([[0, 0, 0, 0]]).T
    tracker.P = np.eye(4) * 500.
    return tracker

# simulate robot movement
N = 30
sensor = PosSensor((0, 0), (2, .2), noise_std=R_std)

zs = np.array([sensor.read() for _ in range(N)])

# run filter
robot_tracker = tracker1()
mu, cov, _, _ = robot_tracker.batch_filter(zs)

for x, P in zip(mu, cov):
    # covariance of x and y
    cov = np.array([[P[0, 0], P[2, 0]], 
                    [P[0, 2], P[2, 2]]])
    mean = (x[0, 0], x[2, 0])
    plot_covariance_ellipse(mean, cov=cov, fc='g', std=3, alpha=0.5)
    