import math
from matplotlib import pyplot as plt
import numpy as np

import matplotlib.pyplot as plt

# Set up the figure and subplots
fig, ax = plt.subplots(1, 2, figsize=(12, 5))
# Add a title to the figure
fig.suptitle("Endpoint calculation measurements (n=10k)")

# Data and labels
jac_labels = ["full jac", "position jac + collision check",
              "projection + collision check", "projection + mesh rotation"]
# dist_simple = [6, 5, 4.5, 4.1]
# dist_full = [31.8, 19.2, 18.4, 17.7]
# nodist_simple = [2.5, 1.5, 1.2, 0.76]
# nodist_full = [14, 1.6, 1.2, 0.76]
dist_simple = [4.9, 5, 4.5]
dist_full = [19.0, 19.2, 18.4]
nodist_simple = [1.5, 1.5, 1.2]
nodist_full = [1.5, 1.6, 1.2]
distance_percentage = 60
cross_size = 100
colours = [(0.8, 0, 0), (0.8, 0.6, 0), (0, 0.3, 0.9), (0.3, 0.9, 0.3)]

# Plotting data for simple mesh calculations
for i in range(len(dist_simple)):
    ax[0].plot([0, 100], [nodist_simple[i], dist_simple[i]], linestyle="--",
               label="Simple mesh: " + jac_labels[i], color=colours[i], linewidth=2)
    y_tgt_simple = (dist_simple[i] - nodist_simple[i]) / \
        100 * distance_percentage + nodist_simple[i]
    ax[0].scatter(distance_percentage, y_tgt_simple,
                  c=colours[i], marker="x", s=cross_size)

# Plotting data for full mesh calculations
for i in range(len(dist_full)):
    ax[1].plot([0, 100], [nodist_full[i], dist_full[i]],
               label="Full mesh: " + jac_labels[i], color=colours[i], linewidth=2)
    y_tgt_full = (dist_full[i] - nodist_full[i]) / \
        100 * distance_percentage + nodist_full[i]
    ax[1].scatter(distance_percentage, y_tgt_full,
                  c=colours[i], marker="x", s=cross_size)

# Setting labels and legends
ax[0].set_xlabel("Distance checking ratio [%]")
ax[0].set_ylabel("Time taken [s]")
ax[0].legend()

ax[1].set_xlabel("Distance checking ratio [%]")
ax[1].set_ylabel("Time taken [s]")
ax[1].legend()

min_time = 0
max_time = max(nodist_simple + dist_simple + nodist_full + dist_full)
max_time = math.ceil(max_time / 10) * 10

ax[0].set_ylim(min_time, max_time)
ax[1].set_ylim(min_time, max_time)

# Display the plot
plt.show()
