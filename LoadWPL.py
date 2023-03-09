"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to load an
array of Waypoints from .txt file.txt The data in .txt file should be in the NMEA format.
"""

import numpy as np


def load_wpl(txt_file: str) -> np.ndarray:
    """ Loads a series of waypoint from a .txt file.

    Args:
        txt_file: path to data file (waypoints must be in the NMEA format). An example of
                  a waypoint is: $MMWPL,5050.710799,N,00044.755897,W,WPT 1.
    Returns:
        array containing the list of waypoints values
    """
    # read the .txt file
    f = None
    try:
        f = open(txt_file, 'r')
    except FileNotFoundError as err:
        print('File name incorrect', err)

    # read the data and output a set of numpy array containing the waypoints
    if f is not None:
        # split into lines
        lines = f.readlines()

        # the array containing the waypoints
        waypoints = []

        # load the waypoints
        for line in lines:
            # check whether the command starts with '$WMPL'
            line_split = line.split(',')
            if line_split[0] == '$MMWPL':
                # append the waypoint data (latitude and longitude)
                waypoints.append([float(line_split[1]), float(line_split[3])])

        # return the numpy array containing the waypoints
        return np.array(waypoints)


if __name__ == '__main__':
    waypoints_test = load_wpl('data.txt')
    print(waypoints_test)
