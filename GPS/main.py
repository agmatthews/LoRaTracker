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
sendInterval = 5 # send data every 5 seconds
ledInterval = 1000 # update LED every 1000usec
dataStructure = '4sBffffffl' # structure for packing data into bytes to send to base unit

# instantiate libraries
pytrack = Pytrack()
l76 = L76GNSS(pytrack)
gps = MicropyGPS(location_formatting='dd') # return decimal degrees from GPS

# instantiate variables
msgSent = False

def GPS_thread():
# continuously reads data from I2C GPS and passes it to micropyGPS for decoding
    global gps
    global l76
    while True:
        NMEAdata = l76._read()
        for c in NMEAdata:
            gps.update(chr(c))

def LED_thread():
# continuously update the status of the unit via the onboard LED
    global gps
    global msgSent
    global ledInterval
    ledColour = 0x000000

    while True:
        if gps.fix_stat > 0:
            # GPS OK so set LED to Green
            ledColour = 0x001100
        else:
            # GPS BAD so set LED to RED
            ledColour = 0x110000

        if gps.time_since_fix()>5000:
            # No GPS fix for last 5 seconds so set LED to Orange
            ledColour = 0x110500

        if gps.clean_sentences < 1:
            # Have not yet had a clean set of data set the LED to Blue
            ledColour = 0x000011

        if msgSent == True:
            # Just sent a message so set the LED to Purple
            ledColour = 0x110011
            #reset the msg flag as we have actioned it
            msgSent = False

        pycom.rgbled(ledColour)
        # Hold the set colour for 90% of the ledInterval
        time.sleep_ms(int(ledInterval * 0.9))
        # Blink LED to black for 10% of the ledInterval to indicate code is still running
        pycom.rgbled(0x000000)
        time.sleep_ms(int(ledInterval * 0.1))

# Startup
print ('Starting GPS (LoRaTracker)')
print ('   ID: ' + str(my_ID))

print ("Starting SD Card")
sd = SD()
os.mount(sd, '/sd')

print ("Starting GPS")
_thread.start_new_thread(GPS_thread, ())
gps.start_logging('/sd/GPS_NMEA_log.txt')

print ("Starting LED")
pycom.heartbeat(False)
pycom.rgbled(0x000011)
_thread.start_new_thread(LED_thread, ())

print ("Starting LoRa")
lora = LoRa(mode=LoRa.LORA, region=LoRa.AU915)
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

# main loop
while True:
    if gps.fix_stat > 0:
        # Got GPS data so send it to base via LoRa
        print('Sending...')
        # get coordinates
        lat = gps.latitude[0]
        lon = gps.longitude[0]
        # correct sign for hemisphere
        if (gps.latitude[1] == 'S'):
            lat = -1 * lat
        if (gps.longitude[1] == 'W'):
            lon = -1 * lon
        # get attributes
        altitude = gps.altitude
        speed = gps.speed[2] #km/h
        course = gps.course
        vBatt = pytrack.read_battery_voltage()
        # get date and time and make an POSIX EPOCH string from it
        GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
        # pack the data into a defined format for tx via lora
        databytes = struct.pack(dataStructure, my_ID, gps.fix_stat, lat, lon, altitude, speed, course, vBatt, GPSdatetime)
        # send the data via LoRa
        s.send(databytes)
        # set msgSent flag to True
        msgSent = True
        # print current data to serial port for debug purposes
        print(str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(gps.date) + ',' + str(vBatt) + ',' + str(gps.timestamp))
    else:
        # no GPS data so just send a ping packet
        print('Pinging...')
        # get date and time and make an POSIX EPOCH string from it
        GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
        vBatt = pytrack.read_battery_voltage()
        # pack the data into a defined format for tx via lora
        databytes = struct.pack(dataStructure, my_ID, gps.fix_stat, 0, 0, 0, 0, 0, vBatt, GPSdatetime)
        # send the data via LoRa
        s.send(databytes)
        # set msgSent flag to False
        msgSent = True

    time.sleep(sendInterval) # wait sendInterval seconds
