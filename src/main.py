import numpy as np
import matplotlib.pyplot as plt

from abs_simulation.vehicle_model import r, slip, dv_vehicle
from abs_simulation.braking_model import T_b, friction_coeff, brake_force, domega_wheel

# init conditions
v_init = 30      
omega_init = v_init / r  
dt = 1e-3         
time_span = 5
time = np.arange(0, time_span, dt)

v_log = [v_init]
omega_log = [omega_init]
lambda_log = [slip(v_init, omega_init)]
mu_log = [friction_coeff(lambda_log[-1])]
F_brake_log = [brake_force(mu_log[-1])]
T_b_log = [T_b]
position_log = [0]

for i in range(1, len(time)):
    v = v_log[-1]
    omega = omega_log[-1]

    # very low speed
    if v <= 1e-3:
        break
    
    lambda_ = slip(v, omega)
    mu = friction_coeff(lambda_)
    F_brake = brake_force(mu)
    
    v_new = v + dv_vehicle(F_brake)
    omega_new = omega + domega_wheel(F_brake)

    # ensure non negative value
    v_new = max(v_new, 0)
    omega_new = max(omega_new, 0)

    # logging
    v_log.append(v_new)
    omega_log.append(omega_new)
    lambda_log.append(lambda_)
    mu_log.append(mu)
    F_brake_log.append(F_brake)
    T_b_log.append(T_b)
    position_log.append(position_log[-1] + v_new * dt)

# truncate logs
time = time[:len(v_log)]
v_log = v_log[:len(time)]
omega_log = omega_log[:len(time)]
lambda_log = lambda_log[:len(time)]
mu_log = mu_log[:len(time)]
F_brake_log = F_brake_log[:len(time)]
T_b_log = T_b_log[:len(time)]
position_log = position_log[:len(time)]

print(f"Braking Distance: {position_log[-1]:.2f} m")
print(f"Braking Time: {time[-1]:.2f} s")

# downsample
plot_every = 10
time_plot = time[::plot_every]
v_plot = v_log[::plot_every]
omega_plot = omega_log[::plot_every]
lambda_plot = lambda_log[::plot_every]
mu_plot = mu_log[::plot_every]
F_brake_plot = F_brake_log[::plot_every]
T_b_plot = T_b_log[::plot_every]
position_plot = position_log[::plot_every]

plt.subplot(2, 3, 1)
plt.plot(time_plot, lambda_plot)
plt.ylabel("Slip Ratio")
plt.xlabel("Time (s)")
plt.grid(True)

plt.subplot(2, 3, 2)
plt.plot(time_plot, mu_plot)
plt.ylabel("Friction Coefficient")
plt.xlabel("Time (s)")
plt.grid(True)

plt.subplot(2, 3, 3)
plt.plot(time_plot, T_b_plot)
plt.ylabel("Brake Torque")
plt.xlabel("Time (s)")
plt.grid(True)

plt.subplot(2, 3, 4)
plt.plot(time_plot, v_plot)
plt.ylabel("Vehicle Speed (m/s)")
plt.xlabel("Time (s)")
plt.grid(True)

plt.subplot(2, 3, 5)
plt.plot(time_plot, omega_plot)
plt.ylabel("Wheel Angular Velocity (rad/s)")
plt.xlabel("Time (s)")
plt.grid(True)

plt.subplot(2, 3, 6)
plt.plot(time_plot, position_plot)
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.grid(True)

plt.tight_layout()
plt.show()
