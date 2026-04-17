import numpy as np

class PIDController:
    def __init__(self, Kp=100, Kd=20, max_force=20):
        self.Kp = Kp
        self.Kd = Kd
        self.max_force = max_force

    def compute(self, x, x_dot, theta, theta_dot):
        # ignore x and x_dot for now
        force = self.Kp * theta + self.Kd * theta_dot
        force = np.clip(force, -self.max_force, self.max_force)
        return force