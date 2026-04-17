import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -------------------------------
# Parameters
# -------------------------------
dt = 0.02
g = 9.81
l = 1.0
m_c = 1.0
m_p = 0.1

# PID gains
Kp = 100
Kd = 20

# State
x, x_dot = 0.0, 0.0
theta, theta_dot = 0.2, 0.0

# -------------------------------
# Plot
# -------------------------------
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-1.5, 1.5)

cart_w, cart_h = 0.4, 0.2
cart = plt.Rectangle((0, -0.1), cart_w, cart_h, color='black')
ax.add_patch(cart)

pole, = ax.plot([], [], lw=3)

# -------------------------------
# Update
# -------------------------------
def update(frame):
    global x, x_dot, theta, theta_dot

    # Controller (still simple PID)
    force = -Kp * theta - Kd * theta_dot

    # Physics
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)

    temp = (force + m_p * l * theta_dot**2 * sin_t) / (m_c + m_p)

    theta_ddot = (g * sin_t - cos_t * temp) / (l * (4/3 - (m_p * cos_t**2) / (m_c + m_p)))

    x_ddot = temp - (m_p * l * theta_ddot * cos_t) / (m_c + m_p)

    # Integrate
    x_dot += x_ddot * dt
    x += x_dot * dt

    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    # Draw
    cart.set_xy((x - cart_w/2, -0.1))

    px = x + l * np.sin(theta)
    py = l * np.cos(theta)

    pole.set_data([x, px], [0, py])

    return pole, cart

ani = FuncAnimation(fig, update, frames=500, interval=20)
plt.title("Real Cart-Pole Dynamics (PID)")
plt.grid()
plt.show()