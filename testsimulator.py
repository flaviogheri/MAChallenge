import math
import matplotlib.pyplot as plt
import numpy as np
import time
from ShipAnim import set_plot


# Create an empty list to store the data
data_x = np.arange(1, 10, 0.1)
data_y = np.arange(1, 10, 0.1)
theta = np.arange(1, 31.4, 0.1)
speed = np.arange(1, 31.4, 0.1)
error = np.arange(1, 31.4, 0.1)

# Initialize the time and frequency variables
t = 0
freq = 1

# Define a function to handle the keyboard interrupt event
def on_key_press(event):
    if event.key == 'p':
        fig.canvas.stop_event_loop()



# waypoint
wp = [[1, 5]]

pl_limits = [[0, 10], [0, 10]]

# Create an empty plot with axis labels
fig, ax = plt.subplots()
plt.ion()

# Connect the keyboard interrupt handler to the plot window
fig.canvas.mpl_connect('key_press_event', on_key_press)

# Continuously generate new data and update the plot
while True:
    #plt.annotate("", xy=(data_x[t]+np.cos(theta[t]), data_y[t]+np.sin(theta[t])), xytext=(data_x[t], data_y[t]),
                #arrowprops=dict(arrowstyle="->"))

    set_plot(waypoints=wp, current_pos = [data_x[t], data_y[t]], current_speed=speed[t], current_err=error[t], limits=pl_limits, axis=ax)

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
