# Plot R2(delta_angle) and R3(z2) as requested.
import numpy as np
import matplotlib.pyplot as plt

# R2(delta_angle) = log(|delta_angle| + 1) + 1
delta_angle = np.linspace(-np.pi, np.pi, 1000)  # radians
R2 = np.log(np.abs(delta_angle) + 1) + 1

plt.figure()
plt.plot(delta_angle, R2)
plt.xlabel("delta_angle (rad)")
plt.ylabel("R2")
plt.title("R2(delta_angle) = log(|delta_angle| + 1) + 1")
plt.grid(True)
plt.show()

# R3(z2) = log(|z2| + 1) / 200 + 0.09
z2 = np.linspace(0, 20, 1000)  # assume distance in meters
R3 = np.log(np.abs(z2) + 1) / 200.0 + 0.09

plt.figure()
plt.plot(z2, R3)
plt.xlabel("z2")
plt.ylabel("R3")
plt.title("R3(z2) = log(|z2| + 1)/200 + 0.09")
plt.grid(True)
plt.show()
