"""
Created by Daniel-Iosif Trubacs on 27 March 2023. The purpose of this module is to create certain functions to help
with the simulation and control of the boat.
"""

import numpy as np


def compare_points(x: np.ndarray, y: np.ndarray) -> bool:
    """ Compares 2 points (lat, lon) and return True if they have the same coordinates."""
    if x[0] == y[0] and x[1] == y[1]:
        return True
    else:
        return False


def next_item(item, array: np.ndarray):
    """ Return the next item from a numpy array of coordinates (lat, lon) when the current item
        (but not index) is known. """
    for i in range(array.shape[0]):
        # look through all the items in the array
        if compare_points(item, array[i]):
            try:
                return array[i + 1]
            except IndexError:
                print("The last item of the array has been reached. Return current item: ", item, " instead of "
                      "the next item.")
                return item


def find_waypoint_name(waypoint, waypoints_list):
    """ Return the name of the current waypoint: WPT plus the index of the waypoint in a given list. """
    # look through all the waypoints in the list
    for i in range(len(waypoints_list)):
        if compare_points(waypoint, waypoints_list[i]):
            return 'WPT' + str(i + 1)

    # if no waypoint found, return None
    return None
