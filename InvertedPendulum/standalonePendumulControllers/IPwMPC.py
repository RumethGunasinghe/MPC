import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -------------------------------
# Parameters
# -------------------------------
dt = 0.01
g = 9.81
l = 1.0
m_c = 1.0
m_p = 0.1

# MPC settings
horizon = 8
force_candidates = np.linspace(-20, 20, 15)

# State
x, x_dot = 0.0, 0.0
theta, theta_dot = 0.2, 0.0

# -------------------------------
# Simulation step (REAL physics)
# -------------------------------
def simulate_step(x, x_dot, theta, theta_dot, force):
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)

    temp = (force + m_p * l * theta_dot**2 * sin_t) / (m_c + m_p)

    theta_ddot = (g * sin_t - cos_t * temp) / (
        l * (4/3 - (m_p * cos_t**2) / (m_c + m_p))
    )

    x_ddot = temp - (m_p * l * theta_ddot * cos_t) / (m_c + m_p)

    # Integrate
    x_dot += x_ddot * dt
    x += x_dot * dt

    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    return x, x_dot, theta, theta_dot


# -------------------------------
# Plot setup
# -------------------------------
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-1.5, 1.5)

cart_w, cart_h = 0.4, 0.2
cart = plt.Rectangle((0, -0.1), cart_w, cart_h, color='black')
ax.add_patch(cart)

pole, = ax.plot([], [], lw=3)
text = ax.text(0.02, 0.95, '', transform=ax.transAxes)


# -------------------------------
# MPC Controller + Update
# -------------------------------
def update(frame):
    global x, x_dot, theta, theta_dot

    best_force = 0
    best_cost = float("inf")

    for force in force_candidates:

        temp_x, temp_x_dot = x, x_dot
        temp_theta, temp_theta_dot = theta, theta_dot

        cost = 0

        for t in range(horizon):

            temp_x, temp_x_dot, temp_theta, temp_theta_dot = simulate_step(
                temp_x, temp_x_dot, temp_theta, temp_theta_dot, force
            )

            # COST FUNCTION
            cost += (
                100 * temp_theta**2        # keep pole upright
                + 1 * temp_theta_dot**2
                + 0.1 * temp_x**2         # reduce drift
            )

        if cost < best_cost:
            best_cost = cost
            best_force = force

    # Apply best force
    x, x_dot, theta, theta_dot = simulate_step(
        x, x_dot, theta, theta_dot, best_force
    )

    # small damping
    x_dot *= 0.999
    theta_dot *= 0.999

    # Draw cart
    cart.set_xy((x - cart_w/2, -0.1))

    # Draw pole
    px = x + l * np.sin(theta)
    py = l * np.cos(theta)
    pole.set_data([x, px], [0, py])

    text.set_text(f"x={x:.2f}, theta={theta:.2f}")

    return pole, cart, text


# -------------------------------
# Run animation
# -------------------------------
ani = FuncAnimation(fig, update, frames=1000, interval=10)

plt.title("Cart-Pole with MPC Control")
plt.grid()
plt.show()