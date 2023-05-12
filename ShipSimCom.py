import serial


# establish serial communication
ser = serial.Serial(port="COM3", baudrate=115200, timeout=1)

def NMEA_CRC(msg):
    """ Calculate the NMEA CRC checksum for a given message """
    #-- We don`t use the $ in the checksum
    mycopy=msg[msg.find("$")+1:] # 150] #99]
    #-- Get the ASC of the first Char
    crc = ord(mycopy[0:1])
    #-- Use a loop to Xor through the string
    for n in range(1,len(mycopy)): #-1):
        crc = crc ^ ord(mycopy[n:n+1])
        #-- Pass the data back as a HEX string
    return '%X' % crc


def set_thrust(ser, thrust):
    "takes as input the desired speed in kts and sends this to the autopilot"

    #cmd = f"$CCAPT,M,{speed},K"
    #checksum = NMEA_CRC(cmd) # calculate checksum of desired command
    #full_cmd = f"{cmd}*{checksum}\r\n" # add checksum to command
    #ser.write(full_cmd.encode()) # write to 
    
    cmd = f"$CCTHD,{thrust},0,0,0,0,0"
    checksum = NMEA_CRC(cmd) # calculate checksum of desired command
    full_cmd = f"{cmd}*{checksum}\r\n" # add checksum to command
    ser.write(full_cmd.encode()) # write to 



def enter_heading_mode(ser):
    """takes as input the desired heading and sets a course for that heading"""
    #-----ENTER AUTO HEADING MODE--------
    cmd = "$CCAPM,7,64,0,80"
    checksum = NMEA_CRC(cmd) # calculate checksum of desired command
    full_cmd = f"{cmd}*{checksum}\r\n" # add checksum to command
    ser.write(full_cmd.encode()) # write to 

def follow_heading(ser, hdg):
    """takes as input the desired heading and sets a course for that heading"""
    
    # ----- FOLLOW DESIRED COMMAND-------
    cmd = f"$CCHSC,{hdg}, T,,"
    checksum = NMEA_CRC(cmd)
    full_cmd = f"{cmd}*{checksum}\r\n"
    ser.write(full_cmd.encode())
    
    
def signal_updates():
    cmd = "$CCNVO,2,1.0,0,0.0," # navigation data output, get data out of port 2, interval between heading fixes. Turning 2 into 0 hides the data
    checksum = NMEA_CRC(cmd)
    full_cmd = f"{cmd}*{checksum}\r\n"
    ser.write(full_cmd.encode())


def decode_response(message):
    """takes as input the message to decode and returns lat, lon, speed, course, utc_time"""
    # decode the byte string to Unicode string
    message = message.decode('utf-8').strip()
    
    # Split the message around the comma
    params = message.split(',')
    
    message_id = params[0][1:] # get the message type

    if message_id == "GPRMC" and len(params) == 14: # if it is a GPS update
        utc_time = params[1]
        lat = params[3]
        lat_dir = params[4]
        lon = params[5]
        lon_dir = params[6]
        speed = params[7] # speed over ground in kts
        course = params[8]
        
        # print(f"UTC time: {utc_time}")
        # print(f"Latitude: {lat} {lat_dir}")
        # print(f"Longitude: {lon} {lon_dir}")
        # print(f"Speed: {speed}")
        # print(f"Course: {course}")
    
        return lat, lat_dir, lon, lon_dir, speed, course, utc_time
    
    else:
        return None

        
ser.close() # close the serial port when done


