"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to establish a
serial connection to Ship Sim 3.
"""

import serial

# establish serial connection
ser = serial.Serial(port='COM4', baudrate=11520, timeout=10)

