"""
Run this file to observe a simulation of a PI controller applied to the case of a reaction
wheel stabilising a rocket's rotational motion.
"""

import numpy as np
import matplotlib.pyplot as plt
import time

# Motor spins at 159 rpm with a torque reduction of 45:1. Thus, without torque reduction, 159*45=7155rpm
max_angular_speed_rw = 7155 * np.pi / 30  # [rad/s]
initial_angular_speed_rocket = 3 * 2*np.pi  # [rad/s] 3 rev/s is what I observed as max spin of rocket

print(f"max_angular_speed_rw: {max_angular_speed_rw}")
print(f"initial_angular_speed_rocket: {initial_angular_speed_rocket}\n")
time.sleep(2)

angular_speed_rocket = initial_angular_speed_rocket
k_p = 1
k_i = 30
k_d = 0
integral = 0
dt = 0.01
ts = np.arange(0, 10, dt)


def get_effect_reaction_wheel(angular_speed_reaction_wheel):
    """
    Simulate effect reaction wheel will have. Consider idealistic scenario where angular momentum is 
    conserved. Reaction wheel effectively absorbs the rocket's angular momentum.
    """
    ratio_inertia_rocket_rw = 25
    angular_speed_rocket = initial_angular_speed_rocket - 1 / ratio_inertia_rocket_rw * angular_speed_reaction_wheel

    return angular_speed_rocket

# Implementation of a simple PI control loop.
list_angular_speed_rocket = [initial_angular_speed_rocket]
for t in ts:
    error = angular_speed_rocket
    p = k_p * error

    integral += error*dt
    i = integral * k_i

    angular_speed_reaction_wheel = p + i
    angular_speed_rocket = get_effect_reaction_wheel(angular_speed_reaction_wheel) # gyroscope

    list_angular_speed_rocket.append(angular_speed_rocket)

print("End")
print(angular_speed_reaction_wheel)
print(angular_speed_rocket)

plt.plot(ts, list_angular_speed_rocket[:-1])
plt.xlabel("Time [s]")
plt.ylabel("Angular Speed of Rocket")
plt.title("PI Controller Simulation")
plt.show()