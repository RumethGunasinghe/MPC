import numpy as np

class MPCHumanoid:
    def __init__(self):
        self.torque_candidates = np.linspace(-100, 100, 15)
        self.horizon = 5
        self.dt = 0.01

    def compute(self, angle, velocity):
        best_torque = 0
        best_cost = float("inf")

        for torque in self.torque_candidates:

            temp_angle = angle
            temp_vel = velocity
            cost = 0

            for _ in range(self.horizon):
                temp_vel += torque * self.dt
                temp_angle += temp_vel * self.dt

                cost += temp_angle**2 + 0.1 * temp_vel**2

            if cost < best_cost:
                best_cost = cost
                best_torque = torque

        return best_torque