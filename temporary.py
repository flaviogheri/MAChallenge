"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to establish a
serial connection to Ship Sim 3.
"""

import serial
import time
# establish serial connection
ser = serial.Serial(port='COM4', baudrate=11520, timeout=1)

#-- Define the NMEA CRC function
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

cmd = "$CCAPM,0,64,0,80"

checksum = NMEA_CRC(cmd)
full_cmd = f"{cmd}*{checksum}\r\n"
ser.write(full_cmd.encode())

response = ser.readline()
print(response)
time.sleep(1)

cmd = "$CCTHD,60,0,0,0,0,0,0,0"

checksum = NMEA_CRC(cmd)
full_cmd = f"{cmd}*{checksum}\r\n"
ser.write(full_cmd.encode())

#print(output_bytes.decode(response))
ser.close()


# byte_string = b'\xff\x10\x05U\xf0\x85\x10\xfbU\n'
# decoded_string = byte_string.decode('utf-8')
# print(decoded_string)