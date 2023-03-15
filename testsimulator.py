import math
import matplotlib.pyplot as plt
import numpy as np

# Create an empty plot with axis labels
fig, ax = plt.subplots()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Amplitude')

# Set the x-axis limits and turn on interactive plotting
plt.xlim([0, 10])
plt.ion()

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

# Connect the keyboard interrupt handler to the plot window
fig.canvas.mpl_connect('key_press_event', on_key_press)

# waypoint
wp = [1, 5]

# Continuously generate new data and update the plot
while True:

    # Generate a new point on the sine wave
    y = math.sin(2 * math.pi * freq * t)

    # Add the new point to the list

    # Update the plot with the new data
    # ax.plot(data, color='blue')
    plt.ylim([0, 10])
    plt.xlim([0, 10])
    ax.plot(wp[0], wp[1], marker = 'o', markersize = 10)
    ax.plot(data_x[t],data_y[t], color = 'r', markersize = 10, marker = '1')
    ax.plot([wp[0], data_x[t]], [wp[1], data_y[t]], linestyle='dashed')

    #plt.arrow(data_x[t],data_y[t], 0.5, 0.5, head_length=0.5)
    plt.annotate("", xy=(data_x[t]+np.cos(theta[t]), data_y[t]+np.sin(theta[t])), xytext=(data_x[t], data_y[t]),
                arrowprops=dict(arrowstyle="->"))
    # Redraw the plot and pause briefly to allow the plot to update
    plt.text(6, 9, f"Heading: {theta[t]:.2f} rad")
    plt.text(6, 8, f"Speed: {speed[t]:.2f} kts")
    plt.text(6, 7, f"Cross Track Error: {error[t]:.2f} m")

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