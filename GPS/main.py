from micropyGPS import MicropyGPS
from network import LoRa
from pytrack import Pytrack
from L76GNSS import L76GNSS
from machine import Timer
import socket
import time
import pycom
import utime
import _thread
import struct
from machine import SD
import os

# configuration
my_ID = 'GPS1' # unique id of this unit - 4 char string

# instantiate libraries
pytrack = Pytrack()
l76 = L76GNSS(pytrack)
gps = MicropyGPS(location_formatting='dd') # return decimal degrees from GPS

def GPS_thread():
# continuously reads data from I2C GPS and passes it to micropyGPS for decoding
    global gps
    global l76
    while True:
        NMEAdata = l76._read()
        for c in NMEAdata:
            gps.update(chr(c))

def DecTime(timeVal):
# convert GPS returned time to seconds past midnight
    time_d = timeVal[0]*60*60 + timeVal[1]*60 + timeVal[2]
    return(time_d)

# Startup
print ('Starting GPS (LoRaTracker)')
print ('   ID: ' + str(my_ID))

print ("Starting LED")
pycom.heartbeat(False)
pycom.rgbled(0x000011)

print ("Starting SD Card")
sd = SD()
os.mount(sd, '/sd')

print ("Starting LoRa")
lora = LoRa(mode=LoRa.LORA, region=LoRa.AU915)
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

print ("Starting GPS")
_thread.start_new_thread(GPS_thread, ())
gps.start_logging('/sd/GPS_NMEA_log.txt')

while True:
    if gps.fix_stat > 0:
        print('Sending...')
        # GPS OK so set LED to Green
        pycom.rgbled(0x001100)
        # get coordinates and attributes
        lat = gps.latitude[0]
        lon = gps.longitude[0]
        # correct sign for hemisphere
        if (gps.latitude[1] == 'S'):
            lat = -1 * lat
        if (gps.longitude[1] == 'W'):
            lon = -1 * lon
        altitude = gps.altitude
        speed = gps.speed[2] #km/h
        course = gps.course
        #get date and time and make an POSIX EPOCH string from it
        GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
        print(str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(gps.date) + ',' + str(gps.timestamp))
        databytes = struct.pack('4sBfffffl', my_ID, gps.fix_stat, lat, lon, altitude, speed, course, GPSdatetime)
        s.send(databytes)
        GPSFix = False
    else:
        print('Pinging...')
        # GPS BAD so set LED to Red
        pycom.rgbled(0x110000)
        GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
        databytes = struct.pack('4sBfffffl', my_ID, gps.fix_stat, 0, 0, 0, 0, 0, GPSdatetime)
        s.send(databytes)

    time.sleep(4.5)
    #blink LED to off to indicate program is still running
    pycom.rgbled(0x000000)
    time.sleep(0.5)
