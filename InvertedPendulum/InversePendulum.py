import numpy as np

class CartPole:
    def __init__(self, theta=0.2, theta_dot=0.0):
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = theta
        self.theta_dot = theta_dot

        self.g = 9.81
        self.l = 1.0
        self.m_c = 1.0
        self.m_p = 0.1
        self.dt = 0.01

    def step(self, force):
        sin_t = np.sin(self.theta)
        cos_t = np.cos(self.theta)

        temp = (force + self.m_p * self.l * self.theta_dot**2 * sin_t) / (self.m_c + self.m_p)

        theta_ddot = (self.g * sin_t - cos_t * temp) / (
            self.l * (4/3 - (self.m_p * cos_t**2) / (self.m_c + self.m_p))
        )

        x_ddot = temp - (self.m_p * self.l * theta_ddot * cos_t) / (self.m_c + self.m_p)

        # Integrate
        self.x_dot += x_ddot * self.dt
        self.x += self.x_dot * self.dt

        self.theta_dot += theta_ddot * self.dt
        self.theta += self.theta_dot * self.dt

        # damping
        self.x_dot *= 0.999
        self.theta_dot *= 0.999

        return self.x, self.theta

    def get_state(self):
        return self.x, self.x_dot, self.theta, self.theta_dot