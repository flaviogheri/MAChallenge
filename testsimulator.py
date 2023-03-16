import matplotlib.pyplot as plt
import numpy as np
from ShipAnim import set_plot
from LoadWPL import load_wpl
from LOS_guidance import DMM_to_DEG
import time

# load the waypoints from data.txt file
tracks = load_wpl('data.txt')
waypoints_list = []
j = 0
for track in tracks:
    for waypoint_dmm in track:
        j += 1
        waypoints_list.append([DMM_to_DEG(waypoint_dmm)[0], DMM_to_DEG(waypoint_dmm)[1]])
        print('waypoint', j, 'dmm', waypoint_dmm, 'deg', DMM_to_DEG(waypoint_dmm))

# change to numpy array
waypoints_list = np.array(waypoints_list)

# Create an empty list to store the data
lat_data = np.arange(50.84514, 50.84518, 0.000001)
lon_data = np.arange(0.745076, 0.745930, 0.000001)

# create random data for speed, error and heading
theta = np.arange(1, 31.4, 0.1)
speed = np.arange(1, 31.4, 0.1)
error = np.arange(1, 31.4, 0.1)

# initial position
initial_position = np.array(DMM_to_DEG(np.array([5050.708799, 44.755897])))

# the range for lat and long for the waypoints and add 10% to it
range_list = np.concatenate((waypoints_list, np.expand_dims(initial_position, axis=0)))

lat_range = 2.5*(np.max(range_list[:,0]) - np.min(range_list[:,0]))
lon_range = 2.5*(np.max(range_list[:,1]) - np.min(range_list[:,1]))



# lat limits for the plot and add 15%
lat_limit = np.array([np.max(range_list[:,0]) - lat_range/2, np.min(range_list[:,0]) + lat_range/2])
lon_limit = np.array([np.max(range_list[:,1]) - lon_range/2, np.min(range_list[:,1]) + lon_range/2])

# the limits for the plot
pl_limits = np.array([lon_limit, lat_limit])

# Initialize the time and frequency variables
t = 0
freq = 1

# Define a function to handle the keyboard interrupt event
def on_key_press(event):
    if event.key == 'p':
        fig.canvas.stop_event_loop()

# Create an empty plot with axis labels
fig, ax = plt.subplots()
plt.ion()

# Connect the keyboard interrupt handler to the plot window
fig.canvas.mpl_connect('key_press_event', on_key_press)

# Continuously generate new data and update the plot
while True:
    #plt.annotate("", xy=(data_x[t]+np.cos(theta[t]), data_y[t]+np.sin(theta[t])), xytext=(data_x[t], data_y[t]),
                #arrowprops=dict(arrowstyle="->"))

    set_plot(waypoints=waypoints_list, current_pos = np.array([lat_data[t], lon_data[-t]]), current_speed=speed[t],
             current_err=error[t], limits=pl_limits, axis=ax)

    plt.draw()
    plt.pause(0.01)

    # Increment the time variable
    t += 1

    # Check if a keyboard interrupt event has occurred
    if not plt.fignum_exists(fig.number):
        break

    # Clear the plot to allow for a live update
    ax.cla()

# Turn off interactive plotting
plt.ioff()

# Show the final plot
plt.show()
