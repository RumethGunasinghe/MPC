import numpy as np

class MPCHumanoid:
    def __init__(self):
        self.torque_candidates = np.linspace(-80, 80, 15)
        self.horizon = 8
        self.dt = 0.01

    def compute(self, angle, angular_vel, linear_vel):

        best_torque = 0
        best_cost = float("inf")

        for torque in self.torque_candidates:

            temp_angle = angle
            temp_ang_vel = angular_vel
            temp_lin_vel = linear_vel

            cost = 0

            for _ in range(self.horizon):

                # 🔥 improved prediction
                temp_ang_vel += torque * self.dt
                temp_angle += temp_ang_vel * self.dt

                temp_lin_vel += torque * 0.5 * self.dt

                # 🔥 COST FUNCTION (VERY IMPORTANT)
                cost += (
                    100 * temp_angle**2 +
                    10 * temp_ang_vel**2 +
                    5 * temp_lin_vel**2
                )

            if cost < best_cost:
                best_cost = cost
                best_torque = torque

        return best_torque