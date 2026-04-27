import numpy as np

class PIDHumanoid:
    def __init__(self):
        self.Kp_angle = 80
        self.Kd_angle = 20

        self.Kp_pos = 30      # 🔥 NEW (COM control)
        self.max_torque = 60

    def compute(self, angle, angular_vel, linear_vel):

        torque = (
            self.Kp_angle * angle +
            self.Kd_angle * angular_vel +
            self.Kp_pos * linear_vel   # 🔥 THIS IS THE BIG UPGRADE
        )

        return np.clip(torque, -self.max_torque, self.max_torque)