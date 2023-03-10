"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to establish a
serial connection to Ship Sim 3.
"""






#-- Define the NMEA CRC function

class ShipSimCom():
    def __init__(self, port):
        self.ser = serial.Serial(port=port, baudrate=9600, timeout=1)
    


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

    def python_to_serial():
        """converts python message to serial message"""

        (heading_input, ) = LOS_latlon()
        heading_output = rad2deg(heading_input)

        output_cmd = "$CCHSC,{:.1f},T,,".format(heading_output)

        return output_cmd


    def serial_to_python():

    

        output_cmd




