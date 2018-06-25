#
from micropyGPS import MicropyGPS
from network import LoRa
from pytrack import Pytrack
from L76GNSS import L76GNSS
from machine import Timer
from machine import WDT
from machine import SD
import socket
import time
import pycom
import utime
import _thread
import struct
import os
import gc
import machine
#from checksum import check_checksum
#from checksum import calc_checksum
from crc16 import crc16xmodem


# configuration
my_ID = '$RM1' # unique id of this unit - 4 char string starting with $
sendInterval = 5 # send data every 5 seconds
ledInterval = 1000 # update LED every 1000usec
dataStructure = '4sBffffffll1s' # structure for packing data into bytes to send to base unit with crc
WDtimeout = int(sendInterval * 4.1 * 1000) # use watchdog timer to reset the board if it does not update reguarly
staleGPStime = 10 # after 10 seconds consider the GPS stale

# instantiate libraries
pytrack = Pytrack()
l76 = L76GNSS(pytrack)
gps = MicropyGPS(location_formatting='dd') # return decimal degrees from GPS
wdt = WDT(timeout=WDtimeout) # enable a Watchdog timer with a specified timeou

# instantiate variables
msgSent = False
time_since_fix = 0
crc = 0
altitude = 0
speed = 0
course = 0
vbatt = 0

def GPS_thread():
# continuously reads data from I2C GPS and passes it to micropyGPS for decoding
    global gps
    global l76
    while True:
        NMEAdata = l76._read()
        for c in NMEAdata:
            result = gps.update(chr(c))
            #if (result != None):
                #print (result)

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

        if gps.time_since_fix()>10:
            # No GPS fix for last 10 seconds so set LED to Orange
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

def readBattery():
    try:
        volts = pytrack.read_battery_voltage()
    except Exception as e:
        print ('Battery read error')
        volts = 0
    return volts

# Startup
print ('Starting GPS (LoRaTracker)')
print ('   ID: ' + str(my_ID))

print ("Starting SD Card")
sd = SD()
os.mount(sd, '/sd')

print ("Starting GPS")
_thread.start_new_thread(GPS_thread, ())

print ("Starting LED")
pycom.heartbeat(False)
pycom.rgbled(0x000011)
_thread.start_new_thread(LED_thread, ())

print ("Starting LoRa")
lora = LoRa(mode=LoRa.LORA, region=LoRa.AU915)
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

print ("Waiting for GPS")
# main loop
while True:
    # Free up memory by garbage collecting
    gc.collect()
    # feed the watch dog timer
    wdt.feed()
    # keep track of time since last fix
    time_since_fix = int(gps.time_since_fix())
    if (time_since_fix is None):
        time_since_fix = -1
    # check we have GPS data and process it if we do
    if gps.fix_stat > 0 and time_since_fix < staleGPStime and time_since_fix >= 0:
        # Got GPS data so send it to base via LoRa
        #print('OK Fix - Sending...')
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
        # check if reading battery can interfere with GPS if occurs at same time as GPS data on I2C bus
        vBatt = readBattery()
        # get date and time and make an POSIX EPOCH string from it
        GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
        # pack the data into a defined format for tx via lora without CRC
        databytes = struct.pack(dataStructure, my_ID, gps.fix_stat, lat, lon, altitude, speed, course, vBatt, GPSdatetime,int(gc.mem_free()),'*')
        # calculate CRC
        #crc = calc_checksum(databytes)
        crc = str(hex(crc16xmodem(databytes)))
        # add the crc to the databytes
        #databytes = databytes + str(hex(crc))
        #databytes = str(hex(crc)) + databytes
        databytes = crc.encode() + databytes
        # send the data via LoRa
        s.send(databytes)
        # set msgSent flag to True
        msgSent = True
        # write received data to log file in CSV format in append mode
        with open("/sd/GPSlog.csv", 'a') as Log_file:
            Log_file.write(my_ID + ',' + str(gc.mem_free()) + ',' + str(gps.fix_stat) + ',' + str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(vBatt) + ',' + str(GPSdatetime) + '\n')
        # print current data to serial port for debug purposes
        print(str(gc.mem_free()) +',TX:,' + str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(gps.date) + ',' + str(vBatt) + ',' + str(gps.timestamp))
        print(databytes)
        print('------')
    else:
        # no GPS data so just send a ping packet
        #print('No Fix - Pinging...')
        # get date and time and make an POSIX EPOCH string from it
        GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
        # read the battery volts
        vBatt = readBattery()
        #check why we are here and calculate gps status
        gpsStat = gps.fix_stat
        if time_since_fix > staleGPStime or time_since_fix < 0:
            gpsStat = 0
        # pack the data into a defined format for tx via lora
        databytes = struct.pack(dataStructure, my_ID, gpsStat, 0, 0, 0, 0, 0, vBatt, GPSdatetime,int(gc.mem_free()),'*')
        # calculate CRC
        #crc = calc_checksum(databytes)
        crc = str(hex(crc16xmodem(databytes)))
        # add the crc to the databytes
        #databytes = databytes + str(hex(crc))
        databytes = crc.encode() + databytes
        # send the data via LoRa
        s.send(databytes)
        # set msgSent flag to False
        msgSent = True
        # print current data for debug purposes
        print(str(gc.mem_free()) +',TX:,0,0,' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(gps.date) + ',' + str(vBatt) + ',' + str(gps.timestamp))
        print(databytes)
        print('------')
    time.sleep(sendInterval) # wait sendInterval seconds
