import numpy as np 
import matplotlib.pyplot as plt

# adjustable constants
R = 10.0  # catapult arm length [m]
phi = 0  # catapult start angle [rad]
phi_stop = np.radians(60)  # catapult stop angle [rad] 
L0 = 0.5  # elastic band undeformed length [m]
k_elas = 9000.0  # elastic spring constant [N/m]
m = 550  # cow mass [kg]
del_x = 300  # range of launch [m]

# catapult function
def catapult(R, phi, phi_stop, L0, k_elas, m):

    # function constants 
    h0 = 60  # height of castle [m]
    rho_cow = 1000  # projectile density [kg/m3]
    c_d = 0.7  # drag coefficient [-]
    rho_air = 1.225  # air density [kg/m3]
    t = 0  # start time [s]
    dt = 0.0001  # time step [s]
    g = 9.81  # gravitational acceleration [m/s^2]
    omega = 0  # start angular velocity
    r_cow = np.cbrt(m / ((4/3) * np.pi * rho_cow))  # cow radius based on mass and density [m]
    A_cow = np.pi * r_cow**2  # cow frontal area [m^2]


    # initialising
    t_tab = []
    x_tab = []
    y_tab = []
    gamma_tab = []
    v_tab = []

    '''launch phase'''
    while phi < phi_stop:
        l_elas = R * np.sin(0.5 * np.pi - phi) / np.sin(0.25 * np.pi + 0.5 * phi)  # length of elastic (from sine rule)
        f_elas = k_elas * (l_elas - L0)  # elastic force
        f_tan = f_elas * np.cos(0.25 * np.pi - 0.5 * phi) - m * g * np.cos(phi)  # tangential components of forces
        # f_rad = - f_elas * np.sin(0.25 * np.pi - 0.5 * phi) + m * g * np.cos(0.5 * np.pi - phi)  # radial components of forces
        
        t += dt  # increment time

        # updating angular quantities
        alpha = f_tan / (m * R)
        omega += alpha * dt
        phi += omega * dt

        # kinematic quantities
        ax = R * alpha * np.cos(0.5 * np.pi - phi)
        ay = R * alpha * np.sin(0.5 * np.pi - phi)
        vx = R * omega * np.cos(0.5 * np.pi - phi)
        vy = R * omega * np.sin(0.5 * np.pi - phi)
        x = -R * np.cos(phi)
        y = R * np.sin(phi)

        # more quantities
        gamma = np.arctan2(vx,vy)  # flight path angle
        v_abs = np.sqrt(vx**2 + vy**2)  # absolute velocity

        # storing values
        t_tab.append(t)
        x_tab.append(x)
        y_tab.append(y)
        gamma_tab.append(gamma)
        v_tab.append(v_abs)

    # print("Launch velocity: ", np.sqrt(vx**2 + vy**2))  # launch velocity

    '''ballisitic trajectory phase'''
    while y > -h0:
        t += dt

        # more quantities
        gamma = np.arctan2(vx,vy)  # flight path angle
        v_abs = np.sqrt(vx**2 + vy**2)  # absolute velocity

        # forces
        drag = c_d * 0.5 * rho_air * v_abs**2 * A_cow
        fx_tot = -drag * np.sin(gamma)
        fy_tot = m * -g -drag * np.cos(gamma)

        # kinematics
        ax = fx_tot / m
        ay = fy_tot / m
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt

        # storing values
        t_tab.append(t)
        x_tab.append(x)
        y_tab.append(y)
        gamma_tab.append(gamma)
        v_tab.append(v_abs)

    distance = x_tab[-1]  # distance travelled [m]

    return x_tab, y_tab, t_tab, gamma_tab, v_tab, distance

# defining a plotting function
def plot(x_tab, y_tab, t_tab, gamma_tab, v_tab, labels, legend):
    empty = [0]  # for our legend so we "plot" something and it shows

    # trajectory plot
    plt.subplot(221)
    plt.title("Trajectory")
    plt.xlabel("Distance [m]")
    plt.ylabel("Height [m]")
    plt.plot(x_tab, y_tab)

    # flight path angle plot
    plt.subplot(222)
    plt.title("Flight Path Angle")
    plt.xlabel("Time [s]")
    plt.ylabel("Flight Path Angle [rad]")
    plt.plot(t_tab, gamma_tab)

    # speed plot
    plt.subplot(223)
    plt.title("Speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.plot(t_tab, v_tab)

    if legend == True:
        # making the legend 
        plt.subplot(224)
        plt.plot(empty)  # plotting the list defined above
        plt.axis("off")  # no axis so it's clean and empty
        plt.title("Legend")
        plt.legend(labels, loc="center")

# selecting the variable that will be varies
var_selection = input("What variable do you want to vary? none / k / L0 / phi_stop: ")

if var_selection == "k":
    labels_k = []  # list for all the k values to add to legend
    # checking for different k values
    for k in range(int(k_elas), 10500, 250):  # iterating through different k values
        x_tab, y_tab, t_tab, gamma_tab, v_tab, distance = catapult(R, phi, phi_stop, L0, k, m)
        labels_k.append(f"k = {k}")  # legend expects a string

        plot(x_tab, y_tab, t_tab, gamma_tab, v_tab, labels_k, legend=True)  # plotting

elif var_selection == "L0":
    labels_l = []
    # checking for different L0 values
    for L in range(10, 0, -2):
        L = L/10  # making back to decimal number
        x_tab, y_tab, t_tab, gamma_tab, v_tab, distance = catapult(R, phi, phi_stop, L, k_elas, m)
        labels_l.append(f"L0 = {L:.1f}")  # legend expects a string
        
        plot(x_tab, y_tab, t_tab, gamma_tab, v_tab, labels_l, legend=True)

elif var_selection == "phi_stop":
    labels_phi = []
    # checking for different phi values
    for phi_st in range(10, 100, 10):
        phi_st = np.radians(phi_st)
        x_tab, y_tab, t_tab, gamma_tab, v_tab, distance = catapult(R, phi, phi_st, L0, k_elas, m)
        labels_phi.append(f"phi_stop = {np.degrees(phi_st):.1f}")  # legend expects a string
        
        plot(x_tab, y_tab, t_tab, gamma_tab, v_tab, labels_phi, legend=True)

elif var_selection == "none":
    x_tab, y_tab, t_tab, gamma_tab, v_tab, distance = catapult(R, phi, phi_stop, L0, k_elas, m)
    labels = []
    
    plot(x_tab, y_tab, t_tab, gamma_tab, v_tab, labels, legend=False)

plt.tight_layout()
plt.show()