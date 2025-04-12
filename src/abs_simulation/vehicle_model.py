import numpy as np

r = 0.3
m = 400

def slip(v_vehicle, omega_wheel):
    if v_vehicle < 1e-3:
        return 0.0  
    slip_ratio = (v_vehicle - omega_wheel * r) / v_vehicle
    return np.clip(slip_ratio, 0.0, 1.0)  

def dv_vehicle(F_brake, dt):
    return (-F_brake / m) * dt
