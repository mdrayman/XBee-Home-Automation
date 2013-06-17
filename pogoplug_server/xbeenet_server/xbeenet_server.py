"""
Continuously read the serial port and process IO data received from a remote XBee.
"""

from xbee import XBee
from xbee import ZigBee
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

# xbee = XBee(ser)

# Use ZigBee() class along with escaped=True since all Xbee radios AP = 2
xbee = ZigBee(ser,shorthand=True,callback=None,escaped=True)

# Continuously read and print packets
while True:
    try:
        response = xbee.wait_read_frame()
        print response
    except KeyboardInterrupt:
        break
        
ser.close()
