import math
from math import radians, degrees, sin, cos, sqrt, atan2
import matplotlib.pyplot as plt
import numpy as np
from LoadWPL import load_wpl
from LOS_guidance import DMM_to_DEG

class local_sim:
    def __init__(self, data_file: str):
        """ Class for Local Simulation"""
        self.wp = None
        self.init_pos = None
        self.theta = None
        self.speed = None
        self.error = None

        self.current_pos = None
        self.current_theta = None
        self.current_speed = None
        self.current_error = None
        self.lat_range = None
        self.lon_range = None
        
        # Create an empty plot with axis labels
        fig, ax = plt.subplots()
        
        self.fig = fig
        self.ax = ax

    def setup_waypoints(self):
        self.wp = [[self.init_pos[0], self.init_pos[1]]]  # initialize with the initial coordinate
        tracks = load_wpl('data.txt')
        for track in tracks:
            for waypoint_dmm in track:
                self.wp.append([DMM_to_DEG(waypoint_dmm)[0], DMM_to_DEG(waypoint_dmm)[1]])
        self.wp = np.array(self.wp)
        for point in self.wp:
            self.ax.plot(point[0], point[1], marker='o', markersize=10)


    def setup_data(self):
        self.data_lat = []
        self.data_lon = []
        self.theta = []
        self.speed = []
        self.error = []

    def on_key_press(self, event):
        # Define a function to handle the keyboard interrupt event
        if event.key == 'p':
            self.fig.canvas.stop_event_loop()


    def init_plot(self):
        """ initiates the plot of function """
        
        # Connect the keyboard interrupt handler to the plot window
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press())


        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')

        self.setup_waypoints()
        # Set the axis limits and turn on interactive plotting
        self.lat_range = np.max(self.wp[:,0]) - np.min(self.wp[:,0])
        self.lon_range = np.max(self.wp[:,1]) - np.min(self.wp[:,1])

        # Add 10% to the range
        self.lat_range *= 1.1
        self.lon_range *= 1.1

        # setup limits
        plt.ylim([init_lat - lat_range, init_lat + lat_range])
        plt.xlim([init_lon - lon_range, init_lon + lon_range])

        self.setup_data()
        plt.ion()
        

    def run_plot(self):
        # Generate a new point on the sine wave
        


        self.ax.plot(self.current_pos[0], self.current_pos[1], color='r', markersize=10, marker='1')

        # plt.arrow(data_x[t],data_y[t], 0.5, 0.5, head_length=0.5)
        plt.annotate("", xy=(self.current_lon+np.cos(self.current_theta), self.current_pos[1] + np.sin(self.current_theta)), xytext=(self.current_lon, self.current_pos[1]),
                    arrowprops=dict(arrowstyle="->"))



        # Redraw the plot and pause briefly to allow the plot to update
        plt.text(self.init_pos[0] + self.lon_range * 0.6, self.init_pos[0] + self.lat_range * 0.9, f"Heading: {theta[t]:.2f} rad")
        plt.text(self.init_pos[0] + self.lat_range * 0.6, self.init_pos[0] + self.lat_range * 0.8, f"Speed: {speed[t]:.2f} kts")
        plt.text(self.init_pos[0] + self.lon_range * 0.6, self.init_pos[0] + self.lat_range * 0.7, f"Cross Track Error: {error[t]:.2f} m")
        plt.draw()
        plt.pause(0.005)

        # Check if a keyboard interrupt event has occurred
        if not plt.fignum_exists(self.fig.number):
            pass

        # Clear the plot to allow for a live update
        ax.cla()

    def end_plot():
        # Turn off interactive plotting
        plt.ioff()

        # Show the final plot
        plt.show()


    def savedata():
        # saves and plots the data of cross track error etc.. on a csv file
        return 0
