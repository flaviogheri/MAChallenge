""" Created by Daniel-Iosif Trubacs and Flavio Gheri for the MAChellenge on 16 March 2023. The main aim of
this module is to create visualization for the ship movement.
"""

import numpy as np
from matplotlib import pyplot as plt


def set_plot(waypoints: np.ndarray, current_pos: np.ndarray, current_speed: float,
             current_err:float, limits:np.ndarray, axis):
    """ Draws the waypoints on interactive plot. Latitude is plotted on the y axis and longitude is plotted on
        the x axis.

    Args:
        waypoints: numpy array representing list of waypoints (n_waypoints, lat ,long)
        current_pos: current position of the bloat (lat, long)
        current_speed: current speed of the bloat
        current_err: current cross track error in relation to the current and last waypoint
        limits: lat and long limits of the current plot [[lat_min,lat_max], [long_min, long_max]]
        axis: matplotlib object representing the axis on which the data is plotted (axis.plot)

    """
    # set the label of the plot
    axis.set_xlabel('Longitude')
    axis.set_ylabel('Latitude')

    # set the limits of the plot
    plt.xlim(limits[0])
    plt.ylim(limits[1])

    # plotting the waypoints
    for i in range(len(waypoints)):
        axis.plot(waypoints[i][1], waypoints[i][0], marker='o', markersize=10)
        axis.text(waypoints[i][1], waypoints[i][0], 'WP'+ str(i+1))

    waypoint_range = [limits[0][1] - limits[0][0], limits[1][1] - limits[1][0]]

    #axis.text(waypoints[0, 1] + waypoint_range[1] * 0.1, waypoints[0, 0] + waypoint_range[0] * 0.05, f"Heading: {heading:.2f} rad")
    axis.text(waypoints[0, 1] - waypoint_range[1] * 0.25, waypoints[0, 0] + waypoint_range[0] * 0.25, f"Speed: {current_speed:.2f} kts")
    axis.text(waypoints[0, 1] - waypoint_range[1] * 0.25, waypoints[0, 0] + waypoint_range[0] * 0.275, f"Cross Track Error: {current_err:.2f} m")
    #print(waypoints[0, 1] + waypoint_range[1] * 0.0, waypoints[0, 0] + waypoint_range[0] * 0.0)
    #print(limits, waypoints[0, 1] + waypoint_range[1] * 0.6)
    # plotting the current position
    #axis.plot(current_pos[1], current_pos[0], color = 'r', markersize=10, marker='1')
    #print(limits[1][1]*0.9999, limits[0][1]*0.9999)
    # Redraw the plot and pause briefly to allow the plot to update


