"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to create
a track from a given set of waypoints. To be run together with LoadWPL and LOS_guidance.
The main run of the simulator should be: update current position (by gps or ship sim) ->
check whether current waypoint has been reached -> update current waypoint -> find heading
"""
"flavio is awseome"
import numpy as np
from LoadWPL import load_wpl
from LOS_guidance import LOS_latlon

class Simulator():
    def __init__(self, data_file: str, waypoints_list: np.ndarray,  current_pos: np.ndarray = None):
        """ Class for simulating ship movement.

        Args:
            data_file: .txt file containing the list of waypoints
            current_pos: current position of the boat (hopefully read by the gps) (defaults to [0,0]
            waypoints_list: numpy array containing list of waypoints
        """
        self.data_file = data_file
        if current_pos is not None:
           self.current_pos = current_pos
        else:
            self.current_pos = np.array([0, 0])
        self.waypoints_list = load_wpl(data_file)

    # a function to find the next waypoint
    def next_waypoint(self, last_waypoint_reached = False, last_waypoint = None, index = None):
        if last_waypoint is None:
            return np.array([self.waypoints_list[0], 0])
        elif last_waypoint_reached == True:
              return np.array([self.waypoints_list[index+1], index+1])
        else:
            np.array([self.waypoints_list[index], index])

    # hopefully used to read from the gps
    def update_position(self) -> np.ndarray:
        # update current position by reading from the gps (or ship sim) not done yet....
        pass

    # find the next heading
    def find_heading(self, current_waypoint, next_waypoint):
        return LOS_latlon(self.current_pos, current_waypoint, next_waypoint)






