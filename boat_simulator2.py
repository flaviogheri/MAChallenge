
import numpy as np
from LoadWPL import load_wpl
from LOS_guidance import LOS_latlon, call_distance, DMM_to_DEG
from ShipSimCom import follow_heading, set_thrust, enter_heading_mode, decode_response
import serial
from bearing_test import bearing
from localsimulator2 import local_sim


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


class local_Simulator:
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
        print("current position: ", self._current_pos)
        
        
    def __update_current_track(self):
        """ Change current track to the next track in the in the list"""
        
        # print("Within __update_current_track function-------------")
        # print("self._current_track = ", self._current_track)
        if self._current_track is None:
            # print("******", self._current_track)
            self._current_track = self.track_list[0]
        else:
            # the index of next track in the tracks list
            next_track_index = self.track_list.index(self._current_track)

            # check whether this is the last trackA
            if next_track_index == len(self.track_list) -1:
                print("This is the last track")

            else:
                # change current track to next track
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
                set_thrust(self._ser, thrust=15)
                if distance_to_wp < 5:
                    self._last_waypoint = self._current_waypoint
                    # print("///////////// READY FOR NEXT WAYPOINT ", next_item(self._current_waypoint, self._current_track))
                    # the next waypoint in current track
                    self._current_waypoint = next_item(self._current_waypoint, self._current_track)
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
            if cross_track_error > 3:
                print("cross track error is over")
            else:
                print("cross track error is within 3")
            return heading
            
            
        else:
            heading = LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[0]
            cross_track_error = LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[1]
            if cross_track_error > 3:
                print("cross track error is over")
            else:
                print("cross track error is within 3")
            return heading
        

    def simulate(self):
        """The main loop running the simulation."""
        
        # create connection with the hardware
        Simulator.create_connection(self, 'COM4', 115200, 1)
        
        set_thrust(self._ser)
        
        enter_heading_mode(self._ser)

        localsim.init_plot()

        # running until the mission is achieved      
        while not self._mission:
            # update position of the boat
            Simulator.__update_position(self)

            # update current track and waypoint
            Simulator.__update_current_track(self)
            # print("--------------", Simulator.__update_current_track(self))
            Simulator.__update_current_waypoint(self)

            # find the next heading for the boat
            heading = Simulator.find_heading(self)
            

            # bearing_value = bearing(self._current_pos, self._current_waypoint)

        
            # print("/////////////////////", heading)

            # implement heading in the boat (send the command to the external hardware)
            
            
            
            
            local_sim.run_plot(self._current_pos, heading, self.current_speed, self.current_error)

            
            

            # check whether the mission has finished (last waypoint has been reached)
            distance = call_distance(self._current_waypoint, self._current_pos)[0]
            if (self._current_waypoint == self.track_list[-1][-1]).all() and distance < 1:
                self._mission = True

                local_sim.end_plot()
                self._ser.close()