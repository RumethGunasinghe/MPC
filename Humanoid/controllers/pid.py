import numpy as np

class PIDHumanoid:
    def __init__(self):
        self.Kp_angle = 80
        self.Kd_angle = 20
        self.Kp_vel = 25
        self.Kp_com = 120   # 🔥 NEW powerful term

        self.max_torque = 80

    def compute(self, angle, ang_vel, lin_vel, com_error):

        torque = (
            self.Kp_angle * angle +
            self.Kd_angle * ang_vel +
            self.Kp_vel * lin_vel +
            self.Kp_com * com_error
        )

        return np.clip(torque, -self.max_torque, self.max_torque)