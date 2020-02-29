# Accelerometer Integration
# Objective: Position detection with velocity derived through acceleration
# Key Components: 1) Model acceleration as measured by the Accelerometer
#                 2) Integrate Acceleration to determine velocity
# Assumptions: - Sample Measured position with 0.01km/hr^2 standard deviation

from soumil_env import KalmanFilter
from soumil_env import Sampler
import numpy as np
import matplotlib.pyplot as plt

class Accelerometer:
    def __init__(self, total_time, time_step):
        self.total_time = total_time
        self.time_step = time_step
        self.no_steps = int(total_time / time_step + 1)
        self.time = np.linspace(0, self.total_time, self.no_steps)

        self.true_acceleration = np.zeros(self.no_steps)
        self.measured_acceleration = np.zeros(self.no_steps)
        self.calculate_acceleration = np.zeros(self.no_steps)
        self.kalman_velocity = np.zeros(self.no_steps)

        self.accceleration_measure = None

        self.error_kalman_true = np.zeros(self.no_steps)
        self.error_calculate_true = np.zeros(self.no_steps)
        self.error_measure_true = np.zeros(self.no_steps)

        self.kalman_filter = KalmanFilter()
        
parser = argparse.ArgumentParser()
parser.add_argument("--sensors", nargs = '+', type = float, help = "position_deviation")