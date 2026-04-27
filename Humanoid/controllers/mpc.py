import numpy as np

class MPCHumanoid:
    def __init__(self):
        self.torque_candidates = np.linspace(-50, 50, 15)
        self.horizon = 5

    def compute(self, torso_angle, torso_velocity):
        best_torque = 0
        best_cost = float("inf")

        for torque in self.torque_candidates:

            temp_angle = torso_angle
            temp_vel = torso_velocity
            cost = 0

            for _ in range(self.horizon):
                # simple model (approx)
                temp_vel += torque * 0.01
                temp_angle += temp_vel * 0.01

                cost += temp_angle**2 + 0.1 * temp_vel**2

            if cost < best_cost:
                best_cost = cost
                best_torque = torque

        return best_torque