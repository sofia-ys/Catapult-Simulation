import numpy as np 

# constants
R = 10.0  # catapult arm length [m]
phiStart = 0  # catapult start angle [rad]
phiStop = np.rad(60)  # catapult stop angle [rad] 
L0 = 0.5  # elastic band undeformed length [m]
kElastic = 9000.0  # elastic spring constant [N/m]
m = 550  # cow mass [kg]

def Vsphere(r):  # volume of a sphere function
    return 4/3 * np.pi() * r**3

