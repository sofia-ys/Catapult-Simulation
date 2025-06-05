import numpy as np
import matplotlib.pyplot as plt

# Constants
g = 9.81  # gravity [m/s²]
rho_air = 1.225  # air density [kg/m³]
Cd = 0.7  # drag coefficient

def simulate_catapult(R, phi_start, phi_stop_deg, L0, k_elastic, m, time_step=0.0001):
    # Initial conditions
    phi_stop = np.radians(phi_stop_deg)
    phi = np.radians(phi_start)
    omega = 0.0  # initial angular velocity
    h0 = 0.0  # height at hinge

    # Launch phase
    t_launch = [0]
    x_launch = [-R * np.cos(phi)]
    y_launch = [h0 + R * np.sin(phi)]
    t = 0

    while phi < phi_stop:
        L = np.sqrt(R**2 + L0**2 - 2 * R * L0 * np.cos(phi))
        F_spring = k_elastic * (L - L0)
        F_tangent = F_spring * np.sin(phi)
        alpha = F_tangent / (m * R)  # angular acceleration

        omega += alpha * time_step
        phi += omega * time_step
        t += time_step
        x_launch.append(-R * np.cos(phi))
        y_launch.append(h0 + R * np.sin(phi))
        t_launch.append(t)

    # Final velocity
    v_launch = omega * R
    vx = v_launch * np.cos(phi)
    vy = v_launch * np.sin(phi)
    print(f"Launch speed: {v_launch:.3f} m/s")

    # Initial ballistic state
    x = x_launch[-1]
    y = y_launch[-1]
    t_ballistic = t
    vx_ball, vy_ball = vx, vy

    # Cow size for drag
    V_sphere = 550 / 1000  # m³ since density is 1000 kg/m³
    r = ((3 * V_sphere) / (4 * np.pi))**(1/3)
    A = np.pi * r**2

    x_ball, y_ball, t_list = [x], [y], [t]

    # Ballistic phase with drag
    dt = 0.001
    while y > -60:
        v = np.sqrt(vx**2 + vy**2)
        F_drag = 0.5 * rho_air * Cd * A * v**2
        ax = -F_drag * vx / (v * m)
        ay = -g - (F_drag * vy / (v * m))

        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        t_ballistic += dt

        x_ball.append(x)
        y_ball.append(y)
        t_list.append(t_ballistic)

    print(f"Distance achieved: {x_ball[-1]:.2f} m")
    return t_list, x_launch + x_ball, y_launch + y_ball, vx, vy

# Plotting wrapper
def plot_trajectory(t, x, y, vx_arr, vy_arr, title='Trajectory'):
    fig, ax = plt.subplots()
    ax.plot(x, y)
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("height [m]")
    ax.grid()
    plt.show()

    # Speed and flight path angle
    speed = np.sqrt(np.array(vx_arr)**2 + np.array(vy_arr)**2)
    angle = np.arctan2(vy_arr, vx_arr) * 180 / np.pi

    fig, axs = plt.subplots(2)
    axs[0].plot(t[:len(speed)], speed)
    axs[0].set_title("Speed vs Time")
    axs[1].plot(t[:len(angle)], angle)
    axs[1].set_title("Flight Path Angle vs Time")
    axs[0].set_ylabel("Speed [m/s]")
    axs[1].set_ylabel("Angle [deg]")
    axs[1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()

# Main function
def main():
    R = 10.0
    phi_start = 0
    phi_stop = 60
    L0 = 0.5
    k_elastic = 9000
    m = 550

    t, x, y, vx_final, vy_final = simulate_catapult(R, phi_start, phi_stop, L0, k_elastic, m)
    plot_trajectory(t, x, y, [vx_final]*len(t), [vy_final]*len(t))

    # Sensitivity analysis
    print("---- Sensitivity Analysis ----")
    for ke in [8000, 9000, 10000]:
        t, x, y, _, _ = simulate_catapult(R, phi_start, phi_stop, L0, ke, m)
        plt.plot(x, y, label=f'k={ke}')
    plt.legend()
    plt.title("Effect of Spring Constant")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()
    plt.show()

    for L in [0.4, 0.5, 0.6]:
        t, x, y, _, _ = simulate_catapult(R, phi_start, phi_stop, L, k_elastic, m)
        plt.plot(x, y, label=f'L0={L}')
    plt.legend()
    plt.title("Effect of L0")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()
    plt.show()

    for angle in [50, 60, 70]:
        t, x, y, _, _ = simulate_catapult(R, phi_start, angle, L0, k_elastic, m)
        plt.plot(x, y, label=f'phi_stop={angle}')
    plt.legend()
    plt.title("Effect of phi_stop")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()
    plt.show()

main()