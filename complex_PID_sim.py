import numpy as np
import matplotlib.pyplot as plt
import time

# Motor spins at 159 rpm with a torque reduction of 45:1. Thus, without torque reduction, 159*45=7155rpm
initial_angular_speed_rocket = 7155 * np.pi / 30  # [rad/s]
print(initial_angular_speed_rocket)
time.sleep(1)

angular_speed_rocket = initial_angular_speed_rocket
k_p = 1
k_i = 20
k_d = 0
integral = 0
dt = 0


def get_effect_reaction_wheel(angular_speed_reaction_wheel):
    angular_speed_rocket = initial_angular_speed_rocket - 1 / 25 * angular_speed_reaction_wheel
    print("")
    print(angular_speed_rocket)
    print(angular_speed_reaction_wheel)

    return angular_speed_rocket

k_p_list  = np.linspace(0, 10, 10)
array_angular_speed_rocket = []

for a,k_p in enumerate(k_p_list):
    print("********************************************************************************************")

    list_angular_speed_rocket = [initial_angular_speed_rocket]
    integral = 0
    for j in range(1999):

        error = angular_speed_rocket
        p = k_p * error

        integral += error*dt
        i = integral * k_i

        print(f"p: {p}")
        print(f"i: {i}")

        angular_speed_reaction_wheel = p + i
        angular_speed_rocket = get_effect_reaction_wheel(angular_speed_reaction_wheel) # gyroscope
        print("")

        list_angular_speed_rocket.append(angular_speed_rocket)

    array_angular_speed_rocket.append(list_angular_speed_rocket)

for i in range(len(array_angular_speed_rocket)):
    plt.plot(array_angular_speed_rocket[i], label=k_p_list[i])
plt.legend()
plt.show()


print(angular_speed_reaction_wheel)
print(angular_speed_rocket)
print("")
