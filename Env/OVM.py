# Optimal Velocity Model

import numpy as np
import matplotlib.pyplot as plt

# Parameters
alpha = 0.5        # sensitivity
V_max = 30.0       # m/s
h_c = 5.0          # minimum safe distance
delta_h = 2.0      # transition sharpness
dt = 0.1           # time step
T = 50             # total time

# Optimal velocity function
def V(h):
    return V_max * np.tanh((h - h_c) / delta_h)

# Initial conditions
x_lead = 0.0
v_lead = 20.0
x_follow = -20.0
v_follow = 15.0

# Storage for plotting
time = np.arange(0, T, dt)
x_f_list, v_f_list = [], []

for t in time:
    # Leader keeps constant speed (for now)
    x_lead += v_lead * dt
    
    # Headway
    h = x_lead - x_follow
    
    # Acceleration from OVM
    a_follow = alpha * (V(h) - v_follow)
    
    # Update follower
    v_follow += a_follow * dt
    x_follow += v_follow * dt
    
    # Store
    x_f_list.append(x_follow)
    v_f_list.append(v_follow)

# Plot follower speed over time
plt.plot(time, v_f_list)
plt.xlabel("Time (s)")
plt.ylabel("Follower Speed (m/s)")
plt.title("Optimal Velocity Model Response")
plt.show()
