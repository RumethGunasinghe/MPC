import numpy as np


class MPCController:

    def __init__(self):

        self.dt = 0.02

        self.horizon = 25

        self.torque_candidates = np.linspace(
            -40,
            40,
            41
        )

    def predict_fall(
        self,
        pitch,
        pitch_vel
    ):

        future_pitch = pitch
        future_vel = pitch_vel

        for _ in range(self.horizon):

            future_vel += 0.01 * future_pitch
            future_pitch += future_vel * self.dt

        return future_pitch

    def compute_balance(
        self,
        pitch,
        pitch_vel
    ):

        best_torque = 0
        best_cost = float("inf")

        for torque in self.torque_candidates:

            temp_pitch = pitch
            temp_vel = pitch_vel

            cost = 0

            for _ in range(self.horizon):

                temp_vel += torque * self.dt
                temp_pitch += temp_vel * self.dt

                cost += (
                    400 * temp_pitch**2
                    + 30 * temp_vel**2
                    + 0.02 * torque**2
                )

            if cost < best_cost:

                best_cost = cost
                best_torque = torque

        return best_torque