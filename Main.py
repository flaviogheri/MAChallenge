"""
Created by Flavio Gheri for the MAC team on 9 March 2023. The purpose of this module is to send the main code
"""

import serial
from math import rad2deg

# establish serial connection
ser = serial.Serial(port='COM4', baudrate=11520, timeout=1)


# Change communication to heading mode
NMEA_CRC("$CCAPM, 7, 64, 0, 80")



cmd = "$CCAPM,0,64,0,80"
cmd = "$CCTHD,25,0,0,0,0,0,0,0"

checksum = NMEA_CRC(cmd)
full_cmd = f"{cmd}*{checksum}\r\n"
ser.write(full_cmd.encode())

response = ser.readline()
print(response)
ser.close()