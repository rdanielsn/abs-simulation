r = 0.3
m = 400

def slip(v_vehicle, omega_wheel):
    return (v_vehicle - omega_wheel * r) / max(v_vehicle, 1e-3)

def dv_vehicle(F_brake, dt):
    return (-F_brake / m) * dt
