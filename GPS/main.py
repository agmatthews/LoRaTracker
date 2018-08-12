import socket
import time
import pycom
import utime
import _thread
import struct
import os
import gc
import machine
import crc16
import ujson
from micropyGPS import MicropyGPS
from network import LoRa
from pytrack import Pytrack
from L76GNSS import L76GNSS
from LIS2HH12 import LIS2HH12
from machine import Timer
from machine import WDT
from machine import SD
from network import WLAN
from microWebSrv import MicroWebSrv
from rgb import RGBLED

# configuration
Unit_ID = '$RM1' # unique id of this unit - 4 char string
staleGPStime = 10 # after 10 seconds consider the GPS stale
accThreshold = 2000 # acceleration threshold to in mG (ie 2000 = 2G)
accDuration = 200 # min acceleration duration in ms
slow_rate = 12 # slow reporting rate GPS to BASE
norm_rate = 10 # normal reporting rate GPS to BASE
fast_rate = 1 # fast reporting rate GPS to BASE
WDtimeout = int(slow_rate * 2.1 * 1000) # use watchdog timer to reset the board if it does not update reguarly
known_nets = { 'Galilean': {'pwd': 'ijemedoo'}, 'lerdy': {'pwd': 'lerdy0519'} , 'tickerg': {'pwd': 'tango papa golf'} }
TZ_offset_secs = 10*60*60  # AEST is +10 hours offset from UTC
ntp_source = 'pool.ntp.org'
webFilePath = '/sd'
ledInterval = 1000  # update LED every 1000ms seconds
ledLightness = 5
batt_check_interval = 60 # check the vattery voltage every 60 Seconds
use_WebServer = True
network_OK = False
lowBatt = 3.5

# Flight velocity thresholds
vTakeOffFW = 15 # FW RPAS must be airborne at 15 kph
vLandingFW = 10 # assume FW RPAS is landed at 10 kph
vTakeOffRW = 6 # RW RPASmust be flying at 5 kph
vLandingRW = 4 # assume RW RPAS is entering hover at 4 kph

# Flight States
FS_UNKNOWN = 0
FS_STATIONARY = 1
FS_TAXI_HOVER = 2
FS_FLIGHT = 3

# set current State and thresholds
currentState = FS_UNKNOWN
vTakeOff = vTakeOffRW
vLanding = vLandingRW

# instantiate variables
msgSent = False
time_since_fix = 0
crc = 0
altitude = 0
speed = 0
course = 0
vBatt = 0
current_rate = norm_rate
last_batt_check = 0
last_send_time = 0
last_LED_check = 0
LED_count = 0

# instantiate libraries
try:
    # Initialize PyTrack
    pytrack = Pytrack()
except Exception as e:
    print("Pytrack initialisation: ERROR")
    time.sleep(0.5)
    machine.reset()
l76 = L76GNSS(pytrack)
acc = LIS2HH12()
gps = MicropyGPS(location_formatting='dd') # return decimal degrees from GPS
wdt = WDT(timeout=WDtimeout) # enable a Watchdog timer with a specified timeou
rtc = machine.RTC()
led = RGBLED(10)

def update_LED():
# continuously update the status of the unit via the onboard LED
    global gps
    global msgSent
    global ledInterval
    global last_LED_check
    global LED_count
    ledColour = 0x000000

    if(utime.ticks_ms() - last_LED_check > ledInterval/10):
        last_LED_check = utime.ticks_ms()
        LED_count += 1
        if (LED_count<9):
            if gps.fix_stat > 0:
                # GPS OK so set LED to Green
                ledHue= led.GREEN
                # GPS OK and Low Battery set LED to YELLOW_GREEN
                if (vBatt<lowBatt):
                    ledHue= led.YELLOW_GREEN
            else:
                # GPS BAD so set LED to RED
                ledHue= led.RED
            if gps.time_since_fix()>10:
                # No GPS fix for last 10 seconds so set LED to Orange
                ledHue= led.ORANGE
            if gps.clean_sentences < 1:
                # Have not yet had a clean set of data set the LED to Blue
                ledHue= led.BLUE
            if msgSent == True:
                # Just sent a message so set the LED to Magenta
                ledHue= led.MAGENTA
                #reset the msg flag as we have actioned it
                msgSent = False
            # set the LED colour with variable lightness based on LED_count
            led.hl(ledHue,LED_count)
        else:
            # blink LED off for 10% of the time
            led.off()
            LED_count = 0

def feed_GPS():
# reads data from I2C GPS and passes it to micropyGPS for decoding
    global gps
    global l76
    GPSdata = l76._read()
    for c in GPSdata:
        gps.update(chr(c))

def readBattery():
    global vBatt
    global last_batt_check
    global batt_check_interval
    if(utime.time() - last_batt_check > batt_check_interval):
        try:
            vBatt = pytrack.read_battery_voltage()
            # keep track of time since last battery check
            last_batt_check = utime.time()
            print ('Battery read: ' + str(vBatt))
            if (vBatt < lowBatt):
                # do more here
                print ('Battery voltage: LOW')

        except Exception as e:
            print ('Battery read: ERROR')
            return
    return

def WWW_routes():
    global gps
    global vBatt
    def _statusjson(client, response):
        statusJSON = {"geometry": {"type": "Point", "coordinates": [str(gps.longitude[0]),str(gps.latitude[0])]}, "type": "Feature", "properties": {"ID": str(Unit_ID), "altitude": str(gps.altitude), "speed": str(gps.speed[2]), "course": str(gps.course), "battery": str(vBatt)}}
        response.WriteResponseJSONOk(statusJSON)
    return [('/status.json', 'GET', _statusjson)]

# Startup
print ('Starting GPS (LoRaTracker)')
print ('   ID: ' + str(Unit_ID))

print ("Starting Network")
wlan = WLAN()
wlan.mode(WLAN.STA)
# scan for available networks
available_nets = wlan.scan()
for net in available_nets:
    print('   Found SSID: '+ net.ssid)
nets = frozenset([e.ssid for e in available_nets])
# match available nets with known nets
known_nets_names = frozenset([key for key in known_nets])
net_to_use = list(nets & known_nets_names)
# try and use the first matching network
try:
    net_to_use = net_to_use[0]
    net_properties = known_nets[net_to_use]
    pwd = net_properties['pwd']
    sec = [e.sec for e in available_nets if e.ssid == net_to_use][0]
    print('   Connecting to: ' + net_to_use)
    if 'wlan_config' in net_properties:
        wlan.ifconfig(config=net_properties['wlan_config'])
    wlan.connect(net_to_use, auth=(sec, pwd), timeout=5000)
    while not wlan.isconnected():
        machine.idle() # save power while waiting
    print('   Connected.')
    print('   IP address: ' + wlan.ifconfig()[0])
    network_OK = True

except Exception as e:
    print('   Cant connect to known networks')
    print('   Entering AP mode')
    wlan.init(mode=WLAN.AP, ssid='GPSnode', channel=6, antenna=WLAN.INT_ANT)

if use_WebServer:
    print ("Starting Webserver")
    routes = WWW_routes()
    mws = MicroWebSrv(routeHandlers=routes, webPath=webFilePath)
    gc.collect()
    mws.Start()
    gc.collect()

print('Starting Clocks')
if network_OK:
    print('   Syncing RTC to '+ ntp_source)
    rtc.ntp_sync(ntp_source)
    utime.sleep_ms(1500)
    print('   RTC Time :', rtc.now())
utime.timezone(TZ_offset_secs)
print('   Local Time:', utime.localtime())

print ("Starting SD Card")
sd = SD()
os.mount(sd, '/sd')
maxIndex = 0
# loop through all the files on the SD card
for f in os.listdir('/sd'):
    #look for GPSlognnnn files
    if f[:6]=='GPSlog':
        try:
            # extract the number from the GPSlognnnn filename
            index = int(f[6:].split(".")[0])
        except ValueError:
            index = 0
        # if this is the highest numbered file then record it
        if index > maxIndex:
            maxIndex = index
if maxIndex>9999:
    print ('   SD card file name error - too many files')
# create a new filename one number higher that the highest on theSD card
log_filename = '/sd/GPSlog{:04d}.csv'.format(maxIndex+1)
# write time and a header row to the new log file
with open(log_filename, 'a') as Log_file:
    Log_file.write(str(rtc.now()) + '\n')
    Log_file.write('Unit_ID,Memory,GPSFix,latitude,longitude,altitude,speed,course,voltage,datetime\n')
print('   Logfile: ' + log_filename)

print ("Starting LED")
led.h(led.ORANGE)

print ("Starting LoRa")
lora = LoRa(mode=LoRa.LORA, region=LoRa.AU915)
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

"""
print ("Starting Accelerometer")
# enable wakeup source from INT pin
pytrack.setup_int_pin_wake_up(False)
# enable activity and also inactivity interrupts, using the default callback handler
pytrack.setup_int_wake_up(True, True)
# enable the activity/inactivity interrupts
acc.enable_activity_interrupt(accThreshold, accDuration)

# check if we were awaken due to activity

if acc.activity():
    #
    pycom.rgbled(0xFF0000)
else:
    #
    pycom.rgbled(0x00FF00)  # timer wake-up


# display the reset reason code and the sleep remaining in seconds
# possible values of wakeup reason are:
# WAKE_REASON_ACCELEROMETER = 1
# WAKE_REASON_PUSH_BUTTON = 2
# WAKE_REASON_TIMER = 4
# WAKE_REASON_INT_PIN = 8

print("Wakeup reason  : " + str(pytrack.get_wake_reason()))
print("sleep remaining: " + str(pytrack.get_sleep_remaining()) + " sec")

"""

print ("Starting GPS")

# main loop
while True:
    # Free up memory by garbage collecting
    gc.collect()
    # feed the watch dog timer
    wdt.feed()
    # update the status LED
    update_LED()
    # process any GPS data on the I2C bus
    feed_GPS()
    # update the battery voltage reading... if needed
    readBattery()
    # if it is time to send a message then send it
    if(utime.time() > last_send_time + current_rate):
        # keep track of time since last fix
        time_since_fix = int(gps.time_since_fix())
        if (time_since_fix is None):
            time_since_fix = -1
        # check we have GPS data and process it if we do
        if gps.fix_stat > 0 and time_since_fix < staleGPStime and time_since_fix >= 0:
            # update local real time clock from GPS if it needs it
            if (rtc.now()[0] < 2018):
                rtc.init((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0))
                print('RTC Time updated from GPS:', rtc.now())
            # get coordinates and attributes
            lat = gps.latitude[0]
            lon = gps.longitude[0]
            altitude = gps.altitude
            speed = gps.speed[2] # format 2 = km/h
            course = gps.course
            # get date and time and make an POSIX EPOCH string from it
            GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
            # pack the data into a dictionary object
            theData = {}
            theData["uid"] = Unit_ID
            theData["typ"] = 'GPS'
            theData["fix"] = gps.fix_stat
            theData["lat"] = lat
            theData["lon"] = lon
            theData["alt"] = altitude
            theData["spd"] = speed
            theData["cog"] = course
            theData["bat"] = vBatt
            theData["gdt"] = GPSdatetime
            theData["mem"] = int(gc.mem_free())
            #convert the dictionionary object to JSON, remove any spaces, and encode it to bytes
            databytes = ujson.dumps(theData).replace(" ", "").encode()
            # calculate CRC and convert it to string with 0 padding to 4 chars
            crc = "0x{:04x}".format(crc16.xmodem(databytes))
            # add the crc to the databytes
            databytes = crc.encode() + databytes            # send the data via LoRa
            s.send(databytes)
            # set msgSent flag to True
            msgSent = True
            # keep track of time since last send
            last_send_time = utime.time()
            # write received data to log file in CSV format in append mode
            with open(log_filename, 'a') as Log_file:
                Log_file.write(Unit_ID + ',' + str(gc.mem_free()) + ',' + str(gps.fix_stat) + ',' + str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(vBatt) + ',' + str(GPSdatetime) + '\n')
            # print current data to serial port for debug purposes
            print (databytes)
        else:
            # no GPS data so just send a ping packet
            #print('No Fix - Pinging...')
            # get date and time and make an POSIX EPOCH string from it
            GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
            #check why we are here and calculate gps status
            gpsStat = gps.fix_stat
            if time_since_fix > staleGPStime or time_since_fix < 0:
                gpsStat = 0
            # pack the data into a dictionary object
            theData = {}
            theData["uid"] = Unit_ID
            theData["typ"] = 'GPS'
            theData["fix"] = gps.fix_stat
            theData["lat"] = 0
            theData["lon"] = 0
            theData["alt"] = 0
            theData["spd"] = 0
            theData["cog"] = 0
            theData["bat"] = vBatt
            theData["gdt"] = GPSdatetime
            theData["mem"] = int(gc.mem_free())
            #convert the dictionionary object to JSON, remove any spaces, and encode it to bytes
            databytes = ujson.dumps(theData).replace(" ", "").encode()
            # calculate CRC
            crc = str(hex(crc16.xmodem(databytes)))
            # add the crc to the databytes
            databytes = crc.encode() + databytes
            # send the data via LoRa
            s.send(databytes)
            # set msgSent flag to False
            msgSent = True
            # keep track of time since last send
            last_send_time = utime.time()
            # print current data for debug purposes
            print (theData)
            #print('------')
        #print('Sleeping for ' + str(current_rate)+ ' seconds')
        #pytrack.setup_sleep(current_rate*1000)
        #pytrack.go_to_sleep()
