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

# configuration
my_ID = '$RM1' # unique id of this unit - 4 char string
dataStructure = '4sBffffffll1s' # structure for packing data into bytes to send to base unit with crc
staleGPStime = 10 # after 10 seconds consider the GPS stale
accThreshold = 2000 # acceleration threshold to in mG (ie 2000 = 2G)
accDuration = 200 # min acceleration duration in ms
slow_rate = 12 # slow reporting rate GPS to BASE
norm_rate = 5 # normal reporting rate GPS to BASE
fast_rate = 1 # fast reporting rate GPS to BASE
WDtimeout = int(slow_rate * 2.1 * 1000) # use watchdog timer to reset the board if it does not update reguarly
known_nets = { 'Galilean': {'pwd': 'ijemedoo'}, 'lerdyx': {'pwd': 'lerdy0519'} }
TZ_offset_secs = 10*60*60  # AEST is +10 hours offset from UTC
ntp_source = 'pool.ntp.org'
webFilePath = '/sd'
ledInterval = 1000  # update LED every 1000ms seconds
ledLightness = 5
batt_check_interval = 60 # check the vattery voltage every 60 Seconds
use_WebServer = True
network_OK = False
lowBatt = 3.5

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
# check if reading battery can interfere with GPS if occurs at same time as GPS data on I2C bus or because of teh sleeps etc
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
    global geoJSON
    def _geojson(client, response):
        response.WriteResponseJSONOk(geoJSON)
    return [('/gps.json', 'GET', _geojson)]

# Startup
print ('Starting GPS (LoRaTracker)')
print ('   ID: ' + str(my_ID))

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
            # keep track of time since last send
            last_send_time = utime.time()
            # get coordinates and attributes
            lat = gps.latitude[0]
            lon = gps.longitude[0]
            altitude = gps.altitude
            speed = gps.speed[2] # format 2 = km/h
            course = gps.course
            # get date and time and make an POSIX EPOCH string from it
            GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
            # pack the data into a defined format for tx via lora without CRC
            databytes = struct.pack(dataStructure, my_ID, gps.fix_stat, lat, lon, altitude, speed, course, vBatt, GPSdatetime,int(gc.mem_free()),'*')
            # calculate CRC
            crc = str(hex(crc16.xmodem(databytes)))
            # add the crc to the databytes
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
            #print(databytes)
            #print('------')
        else:
            # no GPS data so just send a ping packet
            #print('No Fix - Pinging...')
            # get date and time and make an POSIX EPOCH string from it
            GPSdatetime = utime.mktime((int(gps.date[2])+2000, int(gps.date[1]), int(gps.date[0]), int(gps.timestamp[0]), int(gps.timestamp[1]), int(gps.timestamp[2]), 0, 0, 0))
            #check why we are here and calculate gps status
            gpsStat = gps.fix_stat
            if time_since_fix > staleGPStime or time_since_fix < 0:
                gpsStat = 0
            # pack the data into a defined format for tx via lora
            databytes = struct.pack(dataStructure, my_ID, gpsStat, 0, 0, 0, 0, 0, vBatt, GPSdatetime,int(gc.mem_free()),'*')
            # calculate CRC
            #crc = calc_checksum(databytes)
            crc = str(hex(crc16.xmodem(databytes)))
            # add the crc to the databytes
            #databytes = databytes + str(hex(crc))
            databytes = crc.encode() + databytes
            # send the data via LoRa
            s.send(databytes)
            # set msgSent flag to False
            msgSent = True
            # print current data for debug purposes
            print(str(gc.mem_free()) +',TX:,0,0,' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(gps.date) + ',' + str(vBatt) + ',' + str(gps.timestamp))
            #print(databytes)
            #print('------')
        #print('Sleeping for ' + str(current_rate)+ ' seconds')
        #pytrack.setup_sleep(current_rate*1000)
        #pytrack.go_to_sleep()
        #time.sleep(current_rate) # wait current_rate seconds
