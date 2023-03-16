import math
from math import radians, degrees, sin, cos, sqrt, atan2
import matplotlib.pyplot as plt
import numpy as np
from LoadWPL import load_wpl
from LOS_guidance import DMM_to_DEG

# Define the initial coordinate and range
init_lat = 50.8452 #40.7128
init_lon = 0.74594 #-74.0060

wp = [[init_lat, init_lon]]  # initialize with the initial coordinate

tracks = load_wpl('data.txt')
for track in tracks:
    for waypoint_dmm in track:
        wp.append([DMM_to_DEG(waypoint_dmm)[0], DMM_to_DEG(waypoint_dmm)[1]])  
# Create an empty plot with axis labels
fig, ax = plt.subplots()
ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude')
wp = np.array(wp)


lat_range = np.max(wp[:,0]) - np.min(wp[:,0])
lon_range = np.max(wp[:,1]) - np.min(wp[:,1])

# Add 10% to the range
lat_range *= 1.1
lon_range *= 1.1

# Set the axis limits and turn on interactive plotting
plt.ion()

# Create an empty list to store the data
data_lat = np.arange(init_lat - lat_range, init_lat + lat_range, 0.01)
data_lon = np.arange(init_lon - lon_range, init_lon + lon_range, 0.01)
theta = np.arange(1, 31.4, 0.1)
speed = np.arange(1, 31.4, 0.1)
error = np.arange(1, 31.4, 0.1)


# Initialize the time and frequency variables
t = 0
freq = 1

# Define a function to handle the keyboard interrupt event
def on_key_press(event):
    if event.key == 't':
        fig.canvas.stop_event_loop()

# Connect the keyboard interrupt handler to the plot window
fig.canvas.mpl_connect('key_press_event', on_key_press)


# Add the new point to the list


plt.ylim([init_lat - lat_range, init_lat + lat_range])
plt.xlim([init_lon - lon_range, init_lon + lon_range])



# Continuously generate new data and update the plot
while True:

    
    plt.ylim([init_lat - lat_range, init_lat + lat_range])
    plt.xlim([init_lon - lon_range, init_lon + lon_range])
   
    for point in wp:
        ax.plot(point[1], point[0], color= 'red', marker='x', markersize=10)

    # Redraw the plot and pause briefly to allow the plot to update
    # plt.text(init_lon + lon_range * 0.6, init_lat + lat_range * 0.9, f"Heading: {theta[t]:.2f} rad")
    plt.text(init_lon + lon_range * 0.6, init_lat + lat_range * 0.8, f"Speed: {speed[t]:.2f} kts")
    plt.text(init_lon + lon_range * 0.6, init_lat + lat_range * 0.7, f"Cross Track Error: {error[t]:.2f} m")

    plt.draw()
    plt.pause(0.01)

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