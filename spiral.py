import matplotlib.pyplot as plt

inital_coordinates = [5050.708799, 44.755897]
waypoint_n = 15 # number of steps done in the initial search function
x_coord = (5050.710799,5050.732397,5050.732397,5050.732397,5050.710799)
y_coord = (00044.755897,00044.738794,00044.721691,00044.704588)

x_max = max(x_coord)
x_min = min(x_coord)

y_max = max(y_coord)
y_min = min(y_coord)


area_length = max(x_max - x_min, y_max - y_min) # length of total area of square of search pattern

def generate_spiral_square(initial_coordinates, waypoint_n, area_length):
    waypoints = []
    x, y = inital_coordinates
    for i in range(waypoint_n):
        # calculate the next x and y coordinates
        if i % 4 == 0:
            x += area_length
        elif i % 4 == 1:
            y -= area_length
        elif i % 4 == 2:
            x -= area_length
        else:
            y += area_length
        # add the new waypoint to the list
        waypoints.append((x, y))
        # increase the length for the next side
        area_length += 10
    return waypoints

# generate the waypoints
waypoints = generate_spiral_square(inital_coordinates, waypoint_n, area_length)
print(waypoints)


# plot the waypoints
x_values = [x for x, y in waypoints]
y_values = [y for x, y in waypoints]

plt.figure()
plt.plot(x_values, y_values, 'o-')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Spiral Square')

plt.show()


def convert_to_MMWPL(lst):
    message = ""
    for i, tpl in enumerate(lst):
        lat = "{:.6f}".format(abs(tpl[0]))
        lat_dir = "N" if tpl[0] >= 0 else "S"
        lon = "{:.6f}".format(abs(tpl[1]))
        lon_dir = "E" if tpl[1] >= 0 else "W"
        while len(lon.split(".")[0]) < 4:
            lon = "0" + lon
        message += f"$MMWPL,{lat},{lat_dir},0{lon},{lon_dir},WPT {i+1}\n"

    # Add MMRTE line with total number of waypoints
    message += f"$MMRTE,{i+1},1,c,TRACK 1," + ",".join([f"WPT {j+1}" for j in range(i+1)]) + "\n"
    return message


message = convert_to_MMWPL(waypoints)

print(message)
with open("output.txt", "w") as f:
   f.write(message)



