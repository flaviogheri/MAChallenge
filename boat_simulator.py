"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to create
a track from a given set of waypoints. To be run together with LoadWPL and LOS_guidance.
The main loop of the simulation should be:
update current position -> check whether current waypoint has been reached ->
update track and waypoint -> find heading -> set speed -> update current position.
"""

import numpy as np
from LoadWPL import load_wpl
from LOS_guidance import LOS_latlon, call_distance
from ShipSimCom import follow_heading, set_thrust, decode_response
import serial


def next_item(item, array: np.ndarray):
    """ Return the next item from a numpy array when the current item (but not index) is known. """
    index = np.where(item == array)[0][0]
    return array[index + 1]


class Simulator:
    def __init__(self, data_file: str):
        """ Class for simulating ship movement.

        Args:
            data_file: .txt file containing the list of waypoints and tracks
        """
        self.data_file = data_file
        self.track_list = load_wpl(data_file)

        # the current position of the boat, should be updated each time in the loop
        self._current_pos = None

        # the current speed of the boat, should be updated each time in the loop
        self._current_speed = None

        # the current serial object (used to connect to external hardware)
        self._ser = None

        # the current waypoint (next mission for the boat)
        self._current_waypoint = None

        # the current track of the boat
        self._current_track = None

        # the last waypoint for the boat (used to keep the boat on track)
        self._last_waypoint = None

        # a parameter to check whether the last object has been achieved. The main
        # loop should run until this has been achieve
        self._mission = False
        
        self.prev_out = np.zeros(5)

    # a function to create connection with external hardware
    def create_connection(self, n_port: str, n_baudrate: int, n_timeout: int):
        """Establishes serial communication with external hardware. See serial.Serial for more documentation."""

        self._ser = serial.Serial(port=n_port, baudrate=n_baudrate, timeout=n_timeout)
             
        

    def __update_position(self):
        """ Update current position from external readings of GPS."""
        # read current input from serial
        ser_message = self._ser.readline()
        
        if decode_response(ser_message) == None:
            out = self.prev_out

        else:
            # decode message into lat, long, speed, course, utc_time
            out = decode_response(ser_message)
    
        # extract lat and long
        lat = float(out[0])
        long = float(out[2])
        
        # update position of the boat
        self.prev_out = out
        self._current_pos = np.array([lat, long])
        print("current position: ", self._current_pos)
        
        
    def __update_current_track(self):
        """ Change current track when current waypoint becomes last waypoint in the track """
        if self._current_track is None:
            self._current_track = self.track_list[0]
        else:
            # check whether current waypoint is the last waypoint in the track
            if (self._current_waypoint == self._current_track[-1]).all():
                # the index of next track in the tracks list
                next_track_index = self.track_list.index(self._current_track)
                # change current track to next track
                self._current_track = self.track_list[next_track_index]

    def __update_current_waypoint(self):
        """ Update current waypoint """
        
        if self._current_waypoint is None:
            self._current_waypoint = self._current_track[0]
        else:
            # check whether current waypoint has been reached
            distance = call_distance(self._current_waypoint, self._current_pos)[0] # distance in m
            # print("DISTANCE TO WAYPOINT: ", distance)
            if distance < 1:
                # last waypoint becomes current waypoint
                self._last_waypoint = self._current_waypoint

                # the next waypoint in current track
                self._current_waypoint = next_item(self._current_waypoint, self._current_track)
                
        # print("current waypoint", self._current_waypoint)


    # find the next heading
    def find_heading(self):
        # if the boat just started (the first waypoint has not been reached) use [0,0] as start
        # print(self._current_waypoint, "*******")
        print("*/*/*/*/*/*/", self._current_waypoint)
        
        if self._last_waypoint is None:
            heading= LOS_latlon(self._current_pos, self._current_pos, self._current_waypoint)[0]
            return heading
            
        else:
            heading = LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[0]
            return heading
            

    def simulate(self):
        """The main loop running the simulation."""
        
        # create connection with the hardware
        Simulator.create_connection(self, 'COM4', 115200, 1)
        
        set_thrust(self._ser)

        # running until the mission is achieved
        while not self._mission:
            
            # update position of the boat
            Simulator.__update_position(self)

            # update current track and waypoint
            Simulator.__update_current_track(self)

            Simulator.__update_current_waypoint(self)

            # find the next heading for the boat
            heading = Simulator.find_heading(self)*180/np.pi
            # print("/////////////////////", heading)

            # implement heading in the boat (send the command to the external hardware)
            follow_heading(self._ser, heading)

            # check whether the mission has finished (last waypoint has been reached)
            distance = call_distance(self._current_waypoint, self._current_pos)[0]
            if (self._current_waypoint == self.track_list[-1][-1]).all() and distance < 1:
                self._mission = True
                
                self._ser.close()
