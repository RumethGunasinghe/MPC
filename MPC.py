import numpy as np

class MPCController:
    def __init__(self, horizon=8, max_force=20):
        self.horizon = horizon
        self.max_force = max_force
        self.force_candidates = np.linspace(-20, 20, 15)

        # SAME physics params
        self.g = 9.81
        self.l = 1.0
        self.m_c = 1.0
        self.m_p = 0.1
        self.dt = 0.01

    def simulate_step(self, x, x_dot, theta, theta_dot, force):
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)

        temp = (force + self.m_p * self.l * theta_dot**2 * sin_t) / (self.m_c + self.m_p)

        theta_ddot = (self.g * sin_t - cos_t * temp) / (
            self.l * (4/3 - (self.m_p * cos_t**2) / (self.m_c + self.m_p))
        )

        x_ddot = temp - (self.m_p * self.l * theta_ddot * cos_t) / (self.m_c + self.m_p)

        # integrate
        x_dot += x_ddot * self.dt
        x += x_dot * self.dt

        theta_dot += theta_ddot * self.dt
        theta += theta_dot * self.dt

        return x, x_dot, theta, theta_dot

    def compute(self, x, x_dot, theta, theta_dot):
        best_force = 0
        best_cost = float("inf")

        for force in self.force_candidates:

            temp_x, temp_x_dot = x, x_dot
            temp_theta, temp_theta_dot = theta, theta_dot

            cost = 0

            for _ in range(self.horizon):

                temp_x, temp_x_dot, temp_theta, temp_theta_dot = self.simulate_step(
                    temp_x, temp_x_dot, temp_theta, temp_theta_dot, force
                )

                cost += (
                    100 * temp_theta**2
                    + temp_theta_dot**2
                    + 0.1 * temp_x**2
                )

            if cost < best_cost:
                best_cost = cost
                best_force = force

        return best_force