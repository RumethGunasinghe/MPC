import numpy as np

class MPCController:

    def __init__(self):

        self.dt = 0.02

        self.horizon = 20

        self.torque_candidates = np.linspace(
            -50,
            50,
            41
        )

    def compute(
        self,
        angle,
        angular_velocity
    ):

        best_torque = 0

        best_cost = float("inf")

        for torque in self.torque_candidates:

            temp_angle = angle
            temp_vel = angular_velocity

            cost = 0

            for _ in range(self.horizon):

                # simplified pendulum dynamics

                temp_vel += torque * self.dt

                temp_angle += temp_vel * self.dt

                cost += (
                    300 * temp_angle**2
                    + 20 * temp_vel**2
                    + 0.01 * torque**2
                )

            if cost < best_cost:

                best_cost = cost

                best_torque = torque

        return best_torque