"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to create
a track from a given set of waypoints. To be run together with LoadWPL and LOS_guidance.
The main loop of the simulation should be:
update current position -> check whether current waypoint has been reached ->
update track and waypoint -> find heading -> set speed -> update current position.
"""

import numpy as np
from LoadWPL import load_wpl
from LOS_guidance import LOS_latlon, call_distance, DMM_to_DEG
from ShipSimCom import follow_heading, set_thrust, enter_heading_mode, decode_response
import serial
from bearing_test import bearing
import time

def compare_points(x, y):
    if x[0] == y[0] and x[1] == y[1]:
        return True
    else:
        return False

def next_item(item, array: np.ndarray):
    """ Return the next item from a numpy array when the current item (but not index) is known. """
    for i in range(array.shape[0]):
        if compare_points(item, array[i]):
            return array[i+1]
  
wayp = [[5050.710799,   44.755897], [5050.732397,   44.755897], [5050.732397,   44.738794],
        [5050.710799,   44.738794], [5050.710799,   44.721691], [5050.732397,   44.721691],
        [5050.732397,   44.704588], [5050.710799,   44.704588]]
 
def find_waypoint_name(waypoint, waypoints_list=wayp):
    for i in range(len(waypoints_list)):
        if compare_points(waypoint, waypoints_list[i]):
            return 'WPT'+str(i+1)
    
    return 'None'


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
        self._current_track = self.track_list[0]

        # the last waypoint for the boat (used to keep the boat on track)
        self._last_waypoint = None

        # a parameter to check whether the last object has been achieved. The main
        # loop should run until this has been achieve
        self._mission = False
        
        self.prev_out = np.zeros(5)
        
        self.initial_pos = [5050.708799, 44.755897] 

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
        # lat_dir = str(out[1])
        # lon_dir = str(out[3])
        # update position of the boat
        self.prev_out = out
        self._current_pos = np.array([lat, long])
        #print("current position: ", self._current_pos)
        
        
    def __update_current_track(self):
        """ Change current track to the next track in the in the list"""
        
        # print("Within __update_current_track function-------------")
        # print("self._current_track = ", self._current_track)
        if self._current_track is None:
            # print("******", self._current_track)
            print("no track available so chose initial track")
            self._current_track = self.track_list[0]
        else:
            # the index of next track in the tracks list
            print('current track available so go to next track')
            next_track_index = self.track_list.index(self._current_track)+1
            print('next track index', next_track_index)

            # check whether this is the last track
            if next_track_index == len(self.track_list):
                print("This is the last track")

            else:
                # change current track to next track
                print("change to next track/ last track has not been reached")
                self._current_track = self.track_list[next_track_index]
                # print("+++++++++++++ ", self._current_track)



    def __update_current_waypoint(self):
        """ Update current waypoint """
        
        if self._current_waypoint is None:
            self._current_waypoint = self._current_track[0]
        else:
            # check whether current waypoint has been reached
            
            #Convert format of waypoint from DMM to DEG 
            current_waypoint_DEG = DMM_to_DEG(self._current_waypoint)
            current_pos_DEG = DMM_to_DEG(self._current_pos)
            #print("^^^^^^^^^^",self._last_waypoint)
            if self._last_waypoint is not None:
                last_waypoint_DEG = DMM_to_DEG(self._last_waypoint)
            elif self._last_waypoint is None:
                last_waypoint_DEG = DMM_to_DEG(self.initial_pos)

            # print("-----"self._current_waypoint, self._current_pos)
            distance_to_wp = call_distance(current_waypoint_DEG, current_pos_DEG)[0] # distance in m
            distance_from_last_wp = call_distance(last_waypoint_DEG, current_pos_DEG)[0] # distance in m
            #print("DISTANCE TO WAYPOINT: ", distance_to_wp)
            if distance_to_wp < 15 or distance_from_last_wp < 3:
                #print("1kt")
                # last waypoint becomes current waypoint
                set_thrust(self._ser, thrust=10)
                
            if distance_to_wp < 5:
                    print("distance to current is smaller than 5/ change to next waypoint")
                    time.sleep(2)
                    # change last waypoint to current waypoint
                    self._last_waypoint = self._current_waypoint
                    
                    # check whether the current point is the last in the current track
                    if compare_points(self._current_waypoint, self._current_track[-1]):
                        Simulator.__update_current_track(self)
                        self._current_waypoint = self._current_track[0]
                        print("last waypoint in the track reached")
                    else:
                        self._current_waypoint = next_item(self._current_waypoint, self._current_track)
                        print("changing to next waypoint/ still in current track")
            else:
                #print("5kts")
                set_thrust(self._ser, thrust=80)
                
                


    # find the next heading
    def find_heading(self):
        # if the boat just started (the first waypoint has not been reached) use [0,0] as start
        # print(self._current_waypoint, "*******")
        # print("current_waypoint", self._current_waypoint)
        
        if self._last_waypoint is None:
            heading = LOS_latlon(self._current_pos, self.initial_pos, self._current_waypoint)[0]
            cross_track_error = abs(LOS_latlon(self._current_pos, self.initial_pos, self._current_waypoint)[1])
            # print("initial_pos",self.initial_pos)
            print('cross track error:', cross_track_error)
            if cross_track_error > 3:
                print("cross track error is over")
            else:
                print("cross track error is within 3")
            return heading, cross_track_error
            
            
        else:
            heading = LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[0]
            cross_track_error = abs(LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[1])
            print('cross track error:', cross_track_error)
            if cross_track_error > 3:
                print("cross track error is over")
            else:
                print("cross track error is within 3")
            return heading, cross_track_error
        

    def simulate(self):
        """The main loop running the simulation."""
        
        # create connection with the hardware
        Simulator.create_connection(self, 'COM7', 115200, 1)
        
        set_thrust(self._ser)
        
        enter_heading_mode(self._ser)
        
        # average for cross track error
        cros_error_average = 0
        
        # number of values of cross error recorded
        n_cross = 0
        
        # start counting cross track error 
        start_cross = False

        # running until the mission is achieved      
        while not self._mission:
            # update position of the boat
            Simulator.__update_position(self)

            # update current track and waypoint
           # Simulator.__update_current_track(self)
            # print("--------------", Simulator.__update_current_track(self))
            Simulator.__update_current_waypoint(self)
            
            #Convert format of waypoint from DMM to DEG 
            current_waypoint_DEG = DMM_to_DEG(self._current_waypoint)
            current_pos_DEG = DMM_to_DEG(self._current_pos)
            #cross_track_error = LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[1]
            
            # check whether the mission has finished (last waypoint has been reached)
            distance = call_distance(current_waypoint_DEG, current_pos_DEG)[0]
            
            print('current waypoint:', self._current_waypoint)
            print('current pos:', self._current_pos)
            print(find_waypoint_name(self._current_waypoint))
            print('current track:', self.track_list.index(self._current_track))
            print('distance to current waypoint:', distance)
            #print('cross track error:', cross_track_error)

            # find the next heading for the boat
            heading, cross_t_err = Simulator.find_heading(self)

            # bearing_value = bearing(self._current_pos, self._current_waypoint)
            # find average cross track error
            if compare_points(self._current_waypoint, self._current_track[1]):
                start_cross = True
            
            # start counting cross track err
            if start_cross:
             cros_error_average += cross_t_err
             n_cross += 1
            
             print('average cross track error:', cros_error_average/n_cross)
            

        
            # print("/////////////////////", heading)

            # implement heading in the boat (send the command to the external hardware)
            
            follow_heading(self._ser, -heading)

            # check whether current waypoint is the last waypoint in the last track
            if compare_points(self._current_waypoint, self.track_list[-1][-1]):
                print("This is the last waypoint")
                if distance < 5:
                  print("The last waypoint in the last track has been reached")
                  self._ser.close()
                  self._mission = True
                else:
                    print('Distance to last waypoint:', distance)
                
               
