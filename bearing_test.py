import math

def convert_to_decimal_degrees(value):
    value_str = str(value)
    if '.' in value_str:
        degrees = int(value_str[:value_str.index('.')])
        minutes = float(value_str[value_str.index('.'):])
    else:
        degrees = int(value_str[:2])
        minutes = float(value_str[2:])
    decimal_degrees = degrees + (minutes / 60)
    return decimal_degrees

def bearing(point1, point2):
    lat1 = math.radians(convert_to_decimal_degrees(point1[0]))
    lon1 = math.radians(convert_to_decimal_degrees(point1[1]))
    lat2 = math.radians(convert_to_decimal_degrees(point2[0]))
    lon2 = math.radians(convert_to_decimal_degrees(point2[1]))

    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(-(lon2 - lon1))

    bearing = math.degrees(math.atan2(y, x))
    return -(bearing + 360) % 360



# bearing_value = bearing(lat1, lon1, lat2, lon2)
# print(bearing_value)
# 268.0694178094118