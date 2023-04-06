"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to load an
array of Waypoints from a .txt and arrange them in different tracks. A track should be a simple numpy array
containing the waypoints (in order) that should be followed). The data in .txt file should be in the NMEA format.
"""

import numpy as np
import os
import yaml


def load_wpl(txt_file: str) -> list:
    """ Loads a series of waypoint from a .txt file and return the track expected to be followed.

    Args:
        txt_file: path to data file (waypoints must be in the NMEA format). An example of
                  a waypoint is: $MMWPL,5050.710799,N,00044.755897,W,WPT 1. The waypoints will
                  be then arranged in different tracks according to the given commands. An example of
                  of a set track command is: '$MMRTE,2,2,c,TRACK 1,WPT 6,WPT 7,WPT 8'
    Returns:
        (n_tracks, n_waypoints, lat, lon): list containing the list of waypoints values arranged in tracks as numpy arrays
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

        # array containing the names of waypoints (e.g 'WPT 1')
        #  the name of waypoints[i] is given by waypoints_name[i]
        waypoints_name = []

        # load the waypoints
        for line in lines:
            # check whether the command starts with '$WMPL'
            line_split = line.split(',')
            if line_split[0] == '$MMWPL':
                # append the waypoint data (latitude, longitude, waypoint_index)
                waypoints.append(np.array([float(line_split[1]), float(line_split[3])]))
                waypoints_name.append(line_split[5].strip())

        # the array containing all the tracks
        tracks = []

        # go over all the tracks and extract the required waypoints
        # load the waypoints
        for line in lines:
            # check whether the command starts with '$MMRTE'
            line_split = line.split(',')
            if line_split[0] == '$MMRTE':
                # the current waypoints that make up the current track
                track_wp_name = [line_split[i].strip() for i in range(len(line_split)) if line_split[i].strip()
                                 in waypoints_name]

                # the current track containing the waypoints
                current_track = np.array([waypoints[waypoints_name.index(x)] for x in track_wp_name])

                # add the current track to the list of tracks
                tracks.append(current_track)

        # return the numpy array containing the tracks
        return tracks


def update_boat_log(recorded_time: float, recorded_speed: float, recorded_heading: float, recorded_latitude: float,
                    recorded_longitude: float, file_name: str = 'boat_data_log.yaml'):
    """ Saves the speed, heading, position (and the time they were recorded) into a yaml file.

    Args:
        recorded_time: the time at which this data was recorded.
        recorded_speed: recorded speed of the boat by the external hardware.
        recorded_heading: recorded heading of the boat by the external hardware.
        recorded_latitude: recorded latitude of the boat by the external hardware.
        recorded_longitude: recorded longitude of the boat by the external hardware.
        file_name: the name of tha yaml file on which the data is recorded. Defaults to
        'boat_data_log.yaml'

    Returns:

    """
    # if the file exists, load the variables into boat_data_log
    if os.path.exists(file_name):
        # Open the YAML file containing the data for boat setup.
        with open(file_name, "r") as f:
            # Use the load() function to load the file contents into a Python object
            boat_data_log = yaml.load(f, Loader=yaml.FullLoader)

    # if the file doesn't exist, create the boat_data_log variable
    else:
        # Define the initial data for the YAML file
        boat_data_log = {
            "time_log": [],
            "speed_log": [],
            "heading_log": [],
            "position_log": {
                "latitude": [],
                "longitude": []
            }
        }

    # append the data to the log
    boat_data_log["time_log"].append(recorded_time)
    boat_data_log["speed_log"].append(recorded_speed)
    boat_data_log["heading_log"].append(recorded_heading)
    boat_data_log["position_log"]['latitude'].append(recorded_latitude)
    boat_data_log["position_log"]['longitude'].append(recorded_longitude)

    # Write the updated data to the YAML file
    with open(file_name, "w") as yaml_file:
        yaml.dump(boat_data_log, yaml_file)


if __name__ == '__main__':
    tracks_test = load_wpl('data.txt')
    print(tracks_test[0][0])
