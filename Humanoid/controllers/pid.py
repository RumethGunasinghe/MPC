import numpy as np

class PIDHumanoid:
    def __init__(self, Kp=60, Kd=15, max_torque=40):
        self.Kp = Kp
        self.Kd = Kd
        self.max_torque = max_torque

    def compute(self, angle, velocity):
        if abs(angle) < 0.02:
            return 0   # 🔥 ignore small noise

        torque = self.Kp * (angle + 0.02) + self.Kd * velocity
        return np.clip(torque, -self.max_torque, self.max_torque)