import math
from matplotlib import pyplot as plt
import numpy as np

import matplotlib.pyplot as plt

# Set up the figure and subplots

fig, ax = plt.subplots(1, 2, figsize=(10, 4))
# Add a title to the figure
# , $\|\mathtt{bur}\|$=7")
fig.suptitle("Endpoint calculation measurements; bur spines=7")
# Set title for the first subplot
ax[0].set_title("Simple mesh")

# Set title for the second subplot
ax[1].set_title("Full mesh")
# Data and labels
# jac_labels = ["full jac + mesh distance", "full jac", "position jac + collision check",
#   "projection + collision check", "collision checks"]
jac_labels = ["Full jacobian", "Positional jacobian + collision check",
              "Projection + collision check", "Collision checks = bur spines"]  # = $\|\mathtt{bur}\|$"]
# dist_simple = [6, 5, 4.5, 4.1]
# dist_full = [31.8, 19.2, 18.4, 17.7]
# nodist_simple = [2.5, 1.5, 1.2, 0.76]
# nodist_full = [14, 1.6, 1.2, 0.76]
# dist_simple = [6,    4.9,  5,    4.5,  0.2]
# dist_full = [31.8, 19.0, 19.2, 18.4, 0.2]
# nodist_simple = [2.5,  1.5,  1.5,  1.2,  0.2]
# nodist_full = [14,   1.5,  1.6,  1.2,  0.2]
dist_simple = [46.2553,  46.7174,    42.0145,  18.5207]
dist_full = [183.42, 183.863, 179.684, 18.2777]
nodist_simple = [11.4589,  11.8427,  8.44266,  18.2122]
nodist_full = [11.9443,  11.9359,  8.46552,  17.6408]
distance_percentage = 60
cross_size = 100
colours = [(0.8, 0, 0), (0.8, 0.6, 0), (0, 0.3, 0.9),
           (0.3, 0.9, 0.3), (0.5, 0.5, 0.5)]
num_runs = 1e5
time_resolution = 1e6

# Plotting data for simple mesh calculations
for i in range(len(dist_simple)):
    ax[0].plot([0, 100], [nodist_simple[i]/num_runs*time_resolution, dist_simple[i]/num_runs*time_resolution], linestyle="--",
               label=jac_labels[i], color=colours[i], linewidth=2)
    y_tgt_simple = (dist_simple[i] - nodist_simple[i]) / \
        100 * distance_percentage + nodist_simple[i]
    ax[0].scatter(distance_percentage, y_tgt_simple/num_runs*time_resolution,
                  c=colours[i], marker="x", s=cross_size)

# Plotting data for full mesh calculations
for i in range(len(dist_full)):
    ax[1].plot([0, 100], [nodist_full[i]/num_runs*time_resolution, dist_full[i]/num_runs*time_resolution],
               label=jac_labels[i], color=colours[i], linewidth=2)
    y_tgt_full = (dist_full[i] - nodist_full[i]) / \
        100 * distance_percentage + nodist_full[i]
    ax[1].scatter(distance_percentage, y_tgt_full/num_runs*time_resolution,
                  c=colours[i], marker="x", s=cross_size)

# Setting labels and legends
ax[0].set_xlabel("Distance checking ratio [%]")
ax[0].set_ylabel("Time per query [us]")
ax[0].legend()

ax[1].set_xlabel("Distance checking ratio [%]")
ax[1].set_ylabel("Time per query [us]")
ax[1].legend()

min_time = 0
max_time = max(nodist_simple + dist_simple + nodist_full +
               dist_full)/num_runs * time_resolution
max_time = math.ceil(max_time / 100) * 100

ax[0].set_ylim(min_time, max_time)
ax[1].set_ylim(min_time, max_time)

# Display the plot
plt.show()
