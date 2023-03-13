"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to create
certain functions to help with the ship movement visualization
"""

import math
from matplotlib import pyplot as plt
import numpy as np


def animate_ship(current_position: np.ndarray, current_heading: np.ndarray, figure, axis):
    """ Visual feedback for the ship movement

    Args:
        current_position:
        current_heading:
        waypoints:

    Returns:

    """
    axis.set_xlabel('Latitude')
    axis.set_ylabel('Longitude')

    # x and y limits for the plot
    plt.ylim([0, 10])
    plt.xlim([0, 10])

    # plot the position of the ship
    axis.plot(current_position[0], current_position[1], color='r', markersize=10, marker='1')

    # plot the heading of the ship
    axis.annotate("", xy=(current_position[0] + np.cos(current_heading), current_position[1] + np.sin(current_heading)),
                xytext=(current_position[0], current_position[1]), arrowprops=dict(arrowstyle="->"))

    # draw the  plot and pause briefly to allow the plot to update
    plt.draw()
    plt.pause(0.01)

    # clear the plot to allow for a live update
    axis.cla()


# Create an empty list to store the data
data_x = np.arange(1, 10, 0.1)
data_y = np.arange(1, 10, 0.1)
theta = np.arange(1, 31.4, 0.1)

# Initialize the time and frequency variables
t = 0
freq = 1

# turn on interactive plotting
plt.ion()

# Continuously generate new data and update the plot
while True:
    # setup of the plot
    fig, ax = plt.subplots()

    animate_ship(current_position=np.array([data_x[t], data_y[t]]), current_heading=theta[t], figure=fig, axis=ax)

    # Increment the time variable
    t += 1


# Turn off interactive plotting
plt.ioff()

# Show the final plot
plt.show()
