import numpy as np


class Params:
    def __init__(self):
        self.N = 4  # number of state variables
        self.M = 2  # number of control variables
        self.T = 5  # Prediction Horizon
        self.DT = 0.2  # discretization step
        self.path_tick = 0.05
        self.L = 0.3  # vehicle wheelbase
        self.MAX_SPEED = 0.1  # m/s
        self.MAX_ACC = 0.25  # m/ss
        self.MAX_D_ACC = 0.25  # m/sss
        self.MAX_STEER = np.radians(30)  # rad
        self.MAX_D_STEER = np.radians(30)  # rad/s
