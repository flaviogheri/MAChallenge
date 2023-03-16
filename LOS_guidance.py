"""
Created by Chihiro Hirai for the MAC team on 8 March 2023. The purpose of this module is to
calculate the desired heading from current postion, previous waypoint and next waypoint
"""

import math
import numpy as np
from numpy.linalg import norm
from math import sqrt, cos, sin, atan2, pi, degrees


# Convert the unit of Degree Minutes.Minutes to Degrees
def DMM_to_DEG(WP_DMM: np.ndarray):
    """Convert the position data format from DMM(Deg.Minute.Minute) to DEG(Degree)

    Args:
        WP_DMM: Latitude and longitude in DEG format
    Returns:
        Degrees per 1 meter in latitude and longitude direction

    """
    lat = WP_DMM[0]
    lon = WP_DMM[1]

    # For latitude
    DMM_lat_deci,DMM_lat_int=math.modf(lat)
    DMM_lat = str(int(DMM_lat_int)).rjust(4, '0')+str(round(DMM_lat_deci,6)).ljust(8, '0')[1:]
    D1_lat = round(float(str(DMM_lat[0] + DMM_lat[1])))  # Degrees, first two digits
    MM_lat = round(float(DMM_lat[2] + DMM_lat[3])) + round(math.modf(lat)[0], 6)  # Minutes
    D2_lat = round(MM_lat / 60, 6)  # Convert unit from minute to degrees
    DEG_lat = D1_lat + D2_lat

    # For longitude
    DMM_lon_deci,DMM_lon_int=math.modf(lon)
    DMM_lon = str(int(DMM_lon_int)).rjust(5, '0')+str(round(DMM_lon_deci,6)).ljust(8, '0')[1:]
    D1_lon = round(float(str(DMM_lon[0] + DMM_lon[1] + DMM_lon[2])))  # Degrees, first three digits
    MM_lon = round(float(DMM_lon[3] + DMM_lon[4])) + round(math.modf(lon)[0], 6)  # Minutes
    D2_lon = round(MM_lon / 60, 6)  # Convert unit from minute to degrees
    DEG_lon = D1_lon + D2_lon
    return (DEG_lat, DEG_lon)

def call_distance(P1: np.ndarray, P2: np.ndarray):
    """Calculate the distance between two points(DEG format). 

    Args:
        P1: Position 1, Latitude and longitude in DEG format
        P2: Position 2, Latitude and longitude in DEG format
    Returns:
        Degrees per 1 meter in latitude and longitude direction

    """
     
    # Set radius of earth
    pole_radius = 6356752.314245  # Pole radius of earth(m)
    equator_radius = 6378137.0  # Equator radius(m)
    
    #Convert DMM unit to DEG unit 
    #P1=DMM_to_DEG(P1)
    #P2=DMM_to_DEG(P2)

    # Convert latlon to radians
    lat_P1 = math.radians(P1[0])
    lon_P1 = math.radians(P1[1])
    lat_P2 = math.radians(P2[0])
    lon_P2 = math.radians(P2[1])

    lat_difference = lat_P1 - lat_P2  # Difference of latitude
    lon_difference = lon_P1 - lon_P2  # Difference of longitude
    lat_average = (lat_P1 + lat_P2) / 2  # Mean latitude

    e2 = (math.pow(equator_radius, 2) - math.pow(pole_radius, 2)) \
            / math.pow(equator_radius, 2)  # major eccentricity^2

    w = math.sqrt(1 - e2 * math.pow(math.sin(lat_average), 2))

    m = equator_radius * (1 - e2) / math.pow(w, 3)  # meridian curvature radius

    n = equator_radius / w  # prime vertical curvature radius

    distance = math.sqrt(math.pow(m * lat_difference, 2) \
                            + math.pow(n * lon_difference * math.cos(lat_average), 2))  # Distance,m

    return distance, m * lat_difference, n * lon_difference



# Calc for distance on GPS cordinate
def latlon_meter_convertor(P: np.ndarray):
    """Calculate dgrees of 1m in latitude and longitude 

    Args:
        P: Latitude and longitude in DEG format
        
    Returns:
        degrees per 1 meter in latitude and longitude

    """
    # Set radius of earth
    pole_radius = 6356752.314245  # Pole radius of earth
    equator_radius = 6378137.0  # Equator radius
    deg_meter_lon = 360 / (equator_radius * math.cos(math.radians(P[0])) * 2 * np.pi)  # degree/m in longitude
    deg_meter_lat = 360 / (pole_radius * 2 * np.pi)  # degree/m in latitude
    meter_deg_lon = (equator_radius * math.cos(math.radians(P[0])) * 2 * np.pi)/360  # m/degree in longitude
    meter_deg_lat = (pole_radius * 2 * np.pi)/360  # degree/m in latitude
    
    return deg_meter_lat, deg_meter_lon, meter_deg_lat, meter_deg_lon

def LOS_latlon(position: np.ndarray,
               previous_waypoint: np.ndarray,
               current_waypoint: np.ndarray,
               los_radius: float = 15.0,
               debug=False) -> np.ndarray:
    """Calculate the desired angle(degree) to follow the path. 

    Args:
        position: Latitude and longitude in DMM format
        previous_waypoint: Latitude and longitude in DMM format
        current_waypoint: Latitude and longitude in DMM format
        los_radius: control parameter
        debug: whether all parameter should be printed

    Returns:
        heading and error

    """
    # converting the unit from DMM to DEG format
    position = DMM_to_DEG(position)
    previous_waypoint = DMM_to_DEG(previous_waypoint)
    current_waypoint = DMM_to_DEG(current_waypoint)
    deg_meter_lat = latlon_meter_convertor(position)[0] #deg/m latitude
    deg_meter_lon = latlon_meter_convertor(position)[1] #deg/m longitude
    meter_deg_lat = latlon_meter_convertor(position)[2] #m/deg latitude
    meter_deg_lon = latlon_meter_convertor(position)[3] #m/deg longitude


    # ************************Main code for LOS******************************************************
    # Angle of path
    # alpha = atan2((current_waypoint.y-previous_waypoint.y),(current_waypoint.x-previous_waypoint.x))
    #alpha = atan2(call_distance(current_waypoint, previous_waypoint)[1],
    #              call_distance(current_waypoint, previous_waypoint)[2])
    alpha = atan2((current_waypoint[0]-previous_waypoint[0])*meter_deg_lat,
                  (current_waypoint[1]-previous_waypoint[1])*meter_deg_lon)
    # Along-track distance (los_s) and cross-track error (los_e)

    # los_s =((position.x-previous_waypoint.x)*cos(alpha)+(position.y-previous_waypoint.y)*sin(alpha))
    #los_s = ((call_distance(position, previous_waypoint)[2]) * cos(alpha) + (
    #call_distance(position, previous_waypoint)[1]) * sin(alpha))
    los_s =((position[1]-previous_waypoint[1])*meter_deg_lon*cos(alpha)+
            (position[0]-previous_waypoint[0])*meter_deg_lat*sin(alpha))

    # second_angle=atan2(position.y-previous_waypoint.y,position.x-previous_waypoint.x) + alpha
    #second_angle = atan2(call_distance(position, previous_waypoint)[1],
    #                     call_distance(position, previous_waypoint)[2]) + alpha
    second_angle=atan2((position[0]-previous_waypoint[0])*meter_deg_lat,
                       (position[1]-previous_waypoint[1])*meter_deg_lon) + alpha

    # los_e = sqrt((position.x-previous_waypoint.x)**2+(position.y-previous_waypoint.y)**2)*sin(second_angle)
    #los_e = sqrt((call_distance(position, previous_waypoint)[2]) ** 2 + (
    #call_distance(position, previous_waypoint)[1]) ** 2) * sin(second_angle)
    los_e = sqrt(((position[1]-previous_waypoint[1])*meter_deg_lon)**2+
                 ((position[0]-previous_waypoint[0])*meter_deg_lat)**2)*sin(second_angle)


    los_delta = 0.0  # this is correct

    # Compute lookahead distance (los_delta). It is always positive
    if los_radius > abs(los_e):
        los_delta = sqrt(los_radius ** 2 - los_e ** 2)

    # Orthogonal projection (where the blue line turns red)*****working for this
    # xproj = (los_s )*cos(alpha)+previous_waypoint.x
    # yproj = (los_s )*sin(alpha)+previous_waypoint.y
    
    #lon_proj = (los_s + los_delta) * cos(alpha) * deg_meter_lon + previous_waypoint[
    #    1]  # lon projection in degrees + lon of previousWP
    #lat_proj = (los_s + los_delta) * sin(alpha) * deg_meter_lat + previous_waypoint[
    #    0]  # lat projection in degrees + lat of previous WP

    lon_proj = (los_s) * cos(alpha) * deg_meter_lon + previous_waypoint[1]  # lon projection in degrees + lon of previousWP
    lat_proj = (los_s) * sin(alpha) * deg_meter_lat + previous_waypoint[0]  # lat projection in degrees + lat of previous WP

    #TargetP = (lat_proj, lon_proj)
    # Heading point
    # losx = xproj+(los_delta)*cos(alpha) - position.x
    #los_lon = call_distance(TargetP, position)[2]
    los_lon = (lon_proj+los_delta*deg_meter_lon*cos(alpha) - position[1])*meter_deg_lon

    # losy = yproj+(los_delta)*sin(alpha) - position.y
    #los_lat = call_distance(TargetP, position)[1]
    los_lat = (lat_proj+los_delta*deg_meter_lat*sin(alpha) - position[0])*meter_deg_lat
    
    # If the vehicle is beyond the current waypoint, los_s > x:distance between waypoints
    x = np.sqrt(((current_waypoint[0]-previous_waypoint[0])*meter_deg_lat)**2+((current_waypoint[1]-previous_waypoint[1])*meter_deg_lon)**2)
    if los_s > x:
        los_lon = (lon_proj-los_delta*deg_meter_lon*cos(alpha) - position[1])*meter_deg_lon
        los_lat = (lat_proj-los_delta*deg_meter_lat*sin(alpha) - position[0])*meter_deg_lat

    # LOS heading(Desired heading angle in degrees)
    los_heading = degrees(atan2((los_lon), (los_lat)))

    if los_heading < 0:
        los_heading += 360
    elif los_heading > 360:
        los_heading -= 360

    if debug:
        print('Alpha: ', alpha)
        print('los_s: ', los_s)
        print('los_e: ', los_e)
        print('los_delta: ', los_delta)
        print('lat_proj: ', lat_proj)
        print('lon_proj: ', lon_proj)
        print('los_lat: ', los_lat)
        print('los_lon: ', los_lon)
        print('los_heading: ', los_heading)

    # Return LOS heading angle and cross track error
    #print(los_heading)
    return np.array([los_heading, los_e])
    

if __name__ == '__main__':
  P1 = np.array([5050.710799, 00044.755897])
  P2 = np.array([5050.720397, 1044.759597])
  P3 = np.array([5050.732397, 00044.755897])
  wp_degress = DMM_to_DEG([5050.732397, 1044.755897])