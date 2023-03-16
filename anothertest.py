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


# Create an empty list to store the data
data_lat = np.arange(init_lat - lat_range, init_lat + lat_range, 0.01)
data_lon = np.arange(init_lon - lon_range, init_lon + lon_range, 0.01)
theta = np.arange(1, 31.4, 0.1)
speed = np.arange(1, 31.4, 0.1)
error = np.arange(1, 31.4, 0.1)


plt.ylim([init_lat - lat_range, init_lat + lat_range])
plt.xlim([init_lon - lon_range, init_lon + lon_range])



for point in wp:
        print(point[0], point[1])
        plt.plot(point[1], point[0], color= 'red', marker='x', markersize=10)

plt.show()
