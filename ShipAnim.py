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
    axis.set_xlabel('Latitude')
    axis.set_ylabel('Longitude')

    # set the limits of the plot
    plt.xlim(limits[1])
    plt.ylim(limits[0])

    # plotting the waypoints
    for i in range(len(waypoints)):
        axis.plot(waypoints[i][1], waypoints[i][0], marker='o', markersize=10)
        axis.text(waypoints[i][1], waypoints[i][0], 'WP'+str(i))

    # plotting the current position
    axis.plot(current_pos[1], current_pos[0], color = 'r', markersize=10, marker='1')
    axis.text(limits[1][1]*0.8, limits[0][1]*0.9, 'Position:'+str(current_pos[0])+' '+str(current_pos[1]))

    # showing the speed
    axis.text(limits[1][1]*0.8, limits[0][1]*0.8, 'Speed:'+str(round(current_speed, 3)))

    # showing the heading (in degrees)
    axis.text(limits[1][1]*0.8, limits[0][1]*0.7, 'Heading:'+str(round(current_speed, 3)))

