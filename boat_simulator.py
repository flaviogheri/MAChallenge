"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to create
a track from a given set of waypoints. To be run together with LoadWPL and LOS_guidance.
The main loop of the simulation should be:
update current position -> check whether current waypoint has been reached ->
update track and waypoint -> find heading -> set speed -> update current position.
"""

import serial
import time
import yaml
import numpy as np
from LoadWPL import load_wpl, update_boat_log
from LOS_guidance import LOS_latlon, call_distance, DMM_to_DEG
from ShipSimCom import follow_heading, set_thrust, enter_heading_mode, decode_response
from Speed_controller import PID
import matplotlib.pyplot as plt
from ShipAnim import find_limits, set_plot, on_key_press
from boat_control_tools import compare_points, next_item


class Simulator:
    def __init__(self, data_file: str, boat_data_yaml: str):
        """ Class for simulating ship movement.

        All the internal variables (position, speed, current_waypoint, etc) are initialized as None.
        At the start of every simulator all boat characteristics should be  updated from external
        hardware (the Dynautics Simulator) or external files. Moreover, all internal variables should
        be updated every time in the main loop.

        All the hyper parameters (such as acceptance radius, the speed on track, etc) are read from a
        yaml file. The dimensions of the boat should also be present in the yaml file. In the current stage,
        the boat is seen as a point so all dimensions are set to 0.

        Attributes:
            data_file: txt file containing the list of waypoints and tracks. The format should
                       correspond to the one in load_wpl (the DMM format).
            boat_data_yaml: yaml file containing all the boat attributes and hyper parameters necessary
                       for setting the track.
        """
        # the original data file
        self.data_file = data_file

        # the original boat data yaml file containing all the attributes
        self.boat_data_yaml = boat_data_yaml

        # The following variables represent the waypoints and hyper parameters related to them. All
        # this variables are NOT updated during the main loop (simulate method) and their value should stay
        # constant.

        # load the waypoints and tracks
        self.track_list = load_wpl(data_file)

        # Open the YAML file containing the data for boat setup.
        with open(self.boat_data_yaml, "r") as f:
            # Use the load() function to load the file contents into a Python object
            boat_data = yaml.load(f, Loader=yaml.FullLoader)

        # acceptance radius
        self.acceptance_radius = boat_data['hyper_parameters']['acceptance_radius']

        # speed on track
        self.speed_on_track = boat_data['hyper_parameters']['speed']['on_track']

        # speed while turning
        self.speed_while_turning = boat_data['hyper_parameters']['speed']['while_turning']

        # keep speed_on_track before for a certain distance after reaching last waypoint
        self.distance_after_last_waypoint = boat_data['hyper_parameters']['distance']['after_last_waypoint']

        # keep speed_while_turning for a certain distance before reaching current waypoint
        self.distance_before_current_waypoint = boat_data['hyper_parameters']['distance']['before_current_waypoint']

        # initial position as given in ShipSim3. Should be used only for simulation and not boat control.
        # changed the type to numpy array for compatibility
        self._initial_pos = np.array(boat_data['hyper_parameters']['initial_position'])

        # The following variables are all internal variables of the boat. All this variables are
        # updated in the main loop (simulate method).

        # the current position of the boat.
        self._current_pos = None

        # the current speed of the boat.
        self._current_speed = None

        # the current serial object (used to connect to external hardware). Follow the instructions
        # in ShipSimCom for more information.
        self._ser = None

        # the current waypoint (next mission for the boat)
        self._current_waypoint = None

        # the current track of the boat. Initialized as the first track in the given list.
        self._current_track = self.track_list[0]

        # the last waypoint for the boat (used to keep the boat on track)
        self._last_waypoint = None

        # the current heading
        self._current_heading = None

        # the current time at which the external hardware sends data
        self._current_time = None

        # the last recorded time at which the external hardware sent data
        self._last_time = None

        # the previous output of the external hardware (use to have a continuous update even when external hardware
        # is not responding properly)
        self._previous_out = np.zeros(7)

        # a parameter to check whether the last object has been achieved. The main
        # loop should run until this has been achieve
        self._mission = False

    def find_waypoints_deg(self):
        """ Creates an array of waypoints in DEG format from the given list of DMM coordinates. """
        # the waypoints in DEG format
        waypoints_deg = []

        # going through each track in the track list
        for track in self.track_list:
            # going through each waypoint in the track
            for waypoint_dmm in track:
                waypoints_deg.append([DMM_to_DEG(waypoint_dmm)[0], DMM_to_DEG(waypoint_dmm)[1]])

        # return the waypoints
        return np.array(waypoints_deg)

    def create_connection(self, n_port: str, n_baudrate: int, n_timeout: int):
        """ Establishes serial communication with external hardware. See serial.Serial for more documentation. """
        self._ser = serial.Serial(port=n_port, baudrate=n_baudrate, timeout=n_timeout)

    def find_heading(self):
        """ Calculate the required heading of the boat and the current cross_track_error. """
        # Heading and Cross Track Error base on current position, current waypoint and last waypoint.
        heading = LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[0]
        cross_track_error = abs(LOS_latlon(self._current_pos, self._last_waypoint, self._current_waypoint)[1])

        # return the current heading and cross track error
        return heading, cross_track_error

    def __update_position(self):
        """ Update internal attributes from external readings of GPS. """
        # read current input from serial
        ser_message = self._ser.readline()

        # read the message given by the external hardware
        # if the response is None, set all the values to the one given in the last message
        if decode_response(ser_message) is None:
            out = self._previous_out
        # save and decode the message into lat, long, speed, course, utc_time
        else:
            out = decode_response(ser_message)

        # Extract the required attributes from the gps message.
        # latitude at which the boat is positioned
        latitude = float(out[0])

        # longitude at which the boat is positioned
        longitude = float(out[2])

        # the current speed at which the boat is moving
        speed = float(out[4])

        # the time at which the message was sent
        current_time = float(out[6])

        # update the internal attributes with the current values
        self._current_heading = float(out[5])
        self.prev_out = out
        self._current_pos = np.array([latitude, longitude])
        self._current_speed = speed

        # record current and last time
        if self._current_time is not None:
            self._last_time = self._current_time
        self._current_time = current_time

    def __update_current_track(self):
        """ Change current track to the next track in the in the list. """
        # If the boat just started then current track is None. Change to the first track in the list.
        if self._current_track is None:
            self._current_track = self.track_list[0]
        else:
            # the index of next track in the tracks list
            next_track_index = self.track_list.index(self._current_track) + 1

            # check whether this is the last track
            if next_track_index == len(self.track_list):
                print("This is the last track. Current track will remain set to this one.")

            # change the  current track to the next one in the list
            else:
                # change current track to next track
                self._current_track = self.track_list[next_track_index]

    def __update_current_waypoint(self):
        """ Change waypoint if the current waypoint has been achieved. """
        # If the boat just started the current waypoint is None. Change to first waypoint in the track.
        if self._current_waypoint is None:
            self._current_waypoint = self._current_track[0]
        else:

            # if the boat has just started the last waypoint is set to current position
            if self._last_waypoint is not None:
                self._last_waypoint = self._initial_pos

            # distance from current position to current waypoint. This distance is measured in metres
            distance_to_wp = call_distance(self._current_waypoint, self._current_pos, DMM=True)[0]

            # check whether the distance is within a certain distance
            if distance_to_wp < self.acceptance_radius:
                # change last waypoint to current waypoint
                self._last_waypoint = self._current_waypoint

                # check whether the current point is the last in the current track
                # change to next track if the current track is finished
                if compare_points(self._current_waypoint, self._current_track[-1]):
                    Simulator.__update_current_track(self)
                    # update current waypoint to the first waypoint in the next track
                    self._current_waypoint = self._current_track[0]

                # simply change the current waypoint to the next waypoint in the track
                else:
                    self._current_waypoint = next_item(self._current_waypoint, self._current_track)

    def __update_current_speed(self):
        """ Change current speed when the boat is in close proximity to the waypoint. """
        # distance from current position to current waypoint. This distance is measured in metres.
        distance_to_current_waypoint = call_distance(self._current_waypoint, self._current_pos, DMM=True)[0]

        # distance from current position to last waypoint. This distance is measured in metres.
        distance_to_last_waypoint = call_distance(self._current_waypoint, self._current_pos, DMM=True)[0]

        # dt used for PID controlled
        if self._last_time is not None:
            dt = self._current_time - self._last_time
        else:
            dt = self._current_time

        # Use PID controller to change the speed when the boat is in close proximity to the waypoint.
        if distance_to_current_waypoint < self.distance_before_current_waypoint or distance_to_last_waypoint < \
                self.distance_after_last_waypoint:
            # have no idea how this works. More documentation for PID would be amazing.
            if dt > 0:
                controler = PID(Kp=15.0, Ki=0.0, Kd=5.0, setpoint=1.0, limits=(0, 100))  # changed
                PID_output = controler.call(self._current_speed, dt)
                set_thrust(self._ser, PID_output)
        else:
            if dt > 0:
                controler = PID(Kp=15.0, Ki=0.0, Kd=5.0, setpoint=1.0, limits=(0, 100))
                PID_output = controler.call(self._current_speed, dt)
                set_thrust(self._ser, PID_output)

    def simulate(self):
        """ The main loop running the simulation. This loop should run until the last waypoint has been reached.

            This loop follows the order: update current position (read from external hardware) ->
            update current track (check from internal variables of the Simulator class) ->
            update current waypoint (check from internal variables of the Simulator class) ->
            update current speed (send information to external hardware)  ->
            update current heading (send information to external hardware) ->
            update current position (read from external hardware).

            The visualizations part simply shows the position of the boat relative to the waypoints
            given in the mission. The visualization shows all coordinates in DEG format (latitude on
            the y axis and longitude on the x axis), so the distances are not necessarily represented as accurate.
            Other information such as speed, heading and cross track error is shown.
        """

        # create connection with the hardware
        Simulator.create_connection(self, 'COM4', 115200, 1)

        # not sure why we need set_thrust if we have update current speed!!!
        set_thrust(self._ser, thrust=50)

        # no idea what this is
        enter_heading_mode(self._ser)

        # average for cross track error
        cross_error_average = 0

        # number of values of cross track errors recorded
        n_cross = 0

        # start counting cross track error
        start_cross = False

        # the list of waypoints in DEG format
        waypoints_list = Simulator.find_waypoints_deg(self)

        # save the data into a data log
        update_boat_log(recorded_time=self._current_time, recorded_speed=self._current_speed,
                        recorded_heading=self._current_heading, recorded_latitude=self._current_pos[0],
                        recorded_longitude=self._current_pos[1])

        # the limits for the plot.
        plot_limits = find_limits(initial_position=np.array(DMM_to_DEG(self._initial_pos)), waypoints=waypoints_list)

        # Create an empty plot with a figure and axis
        fig, ax = plt.subplots()

        # start interactive plotting
        plt.ion()

        # Connect the keyboard interrupt handler to the plot window
        fig.canvas.mpl_connect('key_press_event', on_key_press)

        # The coordinates followed by the boat in DEG format. Used in plotting the path followed by the boat
        past_lat = []
        past_lon = []

        # running until the mission is achieved
        while not self._mission:
            start_time = time.time()
            # update position of the boat
            Simulator.__update_position(self)

            # update current track and waypoint
            Simulator.__update_current_track(self)

            # update current waypoint
            Simulator.__update_current_waypoint(self)

            # update current speed
            Simulator.__update_current_speed(self)

            # find the next heading and cross track error for the boat
            heading, cross_track_err = Simulator.find_heading(self)

            # implement heading in the boat (send the command to the external hardware)
            follow_heading(self._ser, -heading)

            # start calculating average cross track error only after reaching waypoint 1
            if compare_points(self._current_waypoint, self._current_track[1]):
                start_cross = True

            # find average cross track error
            if start_cross:
                cross_error_average += cross_track_err
                n_cross += 1

            # past coordinates. used for plotting only
            past_lat.append(DMM_to_DEG(self._current_pos[0]))
            past_lon.append(DMM_to_DEG(self._current_pos[1]))

            # the path followed by the boat
            track = np.array([past_lat, past_lon])

            # plot the data on interactive plotting
            set_plot(waypoints=waypoints_list, current_pos=DMM_to_DEG(self._current_pos[0]),
                     current_speed=self._current_speed,
                     current_err=cross_track_err, limits=plot_limits, current_heading=self._current_heading,
                     path=track, axis=ax)

            # show the plot
            plt.draw()
            plt.pause(0.01)

            # Check if a keyboard interrupt event has occurred
            if not plt.fignum_exists(fig.number):
                break

            # Clear the plot to allow for a live update
            ax.cla()
            end_time = time.time()
            print(end_time - start_time)

        # Turn off interactive plotting
        plt.ioff()

        # Show the final plot
        plt.show()

