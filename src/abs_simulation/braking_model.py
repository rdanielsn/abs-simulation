import numpy as np
from abs_simulation.vehicle_model import r, m

# Burckhardt model for dry asphalt
c1 = 1.2801
c2 = 23.99
c3 = 0.52

g = 9.81 
T_b = 1500  
I = 1.5        

dt = 1e-3   

def friction_coeff(lambda_):
    return c1 * (1 - np.exp(-c2 * lambda_)) - c3 * lambda_

def brake_force(mu):
    return mu * m * g

def domega_wheel(F_brake):
    return (-T_b + F_brake * r) / I * dt
