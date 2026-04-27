import numpy as np

class PIDHumanoid:
    def __init__(self, Kp=30, Kd=8, max_torque=30):
        self.Kp = Kp
        self.Kd = Kd
        self.max_torque = max_torque

    def compute(self, angle, velocity):
        if abs(angle) < 0.05:
            return 0   # 🔥 ignore small noise

        torque = self.Kp * angle + self.Kd * velocity
        return np.clip(torque, -self.max_torque, self.max_torque)