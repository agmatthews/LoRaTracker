import pycom
import machine
from network import LoRa
from network import WLAN
from machine import SD
from machine import WDT
from microWebSrv import MicroWebSrv
from mqtt import MQTTClient
import socket
import time
import _thread
import struct
import gc
import ujson
import os
import ubinascii
import uctypes
from checksum import check_checksum
from checksum import calc_checksum
from crc16 import crc16xmodem

# configuration
my_ID = 'BSE1' # unique id of this unit - 4 char string
receiveInterval = 2 # send data every 2 seconds
ledInterval = 1000 # update LED every 1000usec
WLAN_SSID = 'lerdy'
WLAN_PWD = 'lerdy0519'
dataStructure = '4sBffffffll1s' # structure packed data bytes received
RockAirInterval = 2 * 60 * 1000  # Send data to TracPlus every (2 * 60 * 1000)usec = 2 minutes
WDtimeout = int(25 * 1000) # use watchdog timer to reset the board if it does not update reguarly (25 seconds)

# istantiate libraries
wdt = WDT(timeout=WDtimeout) # enable a Watchdog timer with a specified timeou

# initialise variables
GPSFix = False
FixTimeout = 1000 * 30  # 30 seconds in ms
lastFix = time.time() - FixTimeout
GPSdatetime = None
lat = None
lon = None
altitude  = None
speed = None
course = None
remote_ID = b''
vBatt = 0.0
CRC = b''
msgReceived = False
geoJSON = {}
remoteMem = 0

def LED_thread():
# continuously update the status of the unit via the onboard LED
    global GPSFix
    global msgReceived
    global ledInterval
    ledColour = 0x000000

    while True:
        if GPSFix > 0:
            # GPS OK so set LED to Green
            ledColour = 0x001100
        else:
            # GPS BAD so set LED to RED
            ledColour = 0x110000

        if msgReceived == True:
            # Just Rx a message so set the LED to Purple
            ledColour = 0x110011
            #reset the msg flag as we have actioned it
            msgReceived = False

        pycom.rgbled(ledColour)
        # Hold the set colour for 90% of the ledInterval
        time.sleep_ms(int(ledInterval * 0.9))
        # Blink LED to black for 10% of the ledInterval to indicate code is still running
        pycom.rgbled(0x000000)
        time.sleep_ms(int(ledInterval * 0.1))

def DataSend_thread():
# periodically send data to TracPlus via RockAir via Serial Port
# TESTING FTDI port /dev/cu.usbserial-A800JWP3
    global GPSFix
    global RockAirInterval

    while True:
        if GPSFix > 0:
            # GPS OK so send a message
            print ("GPS FIX OK - so send a message")
            # Free up memory by garbage collecting
            gc.collect()
            # send data out on serial port
            print ("ask RockAir for location")
            #uart1.write(remote_ID + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(vBatt) + ',' + str(stats.rssi) + str(GPSdatetime) + '\n')
            uart1.write('R7+GPS\r')
            print ("get location from RockAir")
            print(uart1.readline())
            # send data to MQTT server
            mqtt.publish(topic="agmatthews/feeds/LORAtest", msg=remote_ID + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(GPSdatetime) + ',' + str(stats.rssi))

        #else:
            # GPS BAD so don't send message
            #
            #print ("NO GPS FIX - so don't send message")

        time.sleep_ms(int(RockAirInterval))

def mqtt_callback(topic, msg):
    # this subroutine would process any message that comes back from MQTT server
    print(msg)

def WWW_routes():
    global geoJSON
    def _geojson(client, response):
        response.WriteResponseJSONOk(geoJSON)
    return [('/gps.json', 'GET', _geojson)]

print ('Starting BASE (LoRaTracker)')
print ('   ID: ' + str(my_ID))

print ("Starting Network")
wlan = WLAN(mode=WLAN.STA)
nets = wlan.scan()
for net in nets:
    print('   Found SSID: '+ net.ssid)
    if net.ssid == WLAN_SSID:
        print('   Connecting to: '+ net.ssid)
        wlan.connect(net.ssid, auth=(net.sec, WLAN_PWD), timeout=5000)
        while not wlan.isconnected():
            machine.idle() # save power while waiting
        print("   Connected IP address:" + wlan.ifconfig()[0])
        break

print ("Starting LED")
pycom.heartbeat(False)
pycom.rgbled(0x000011)
_thread.start_new_thread(LED_thread, ())

print ("Starting DataSend")
_thread.start_new_thread(DataSend_thread, ())

print ("Starting UART1")
uart1 = UART(1, 19200, bits=8, parity=None, stop=1)
uart1.init(baudrate=19200, bits=8, parity=None, stop=1)
# send data out on serial port
print ("   query RockAir for location")
uart1.write('R7+GPS\r')
time.sleep(1)
# get something back
RockAirResponse = b''
while uart1.any():
    RockAirResponse = uart1.readline()
    if len(RockAirResponse)>5:
        break
if len(RockAirResponse)>5:
    #turn response into string
    RockAirResponse = RockAirResponse.decode()
    # strip off any carridge returns or new line characters
    RockAirResponse = RockAirResponse.replace('\r', '')
    RockAirResponse = RockAirResponse.replace('\n', '')
    # split the response into segments at each comma
    response_segments = str(RockAirResponse).split(',')
    print (response_segments)

    # Latitude
    r_string = response_segments[1]
    lat_degs = int(r_string[0:2])
    lat_mins = float(r_string[2:-2])
    lat_hemi = r_string[-1]
    lat_dec = lat_degs + lat_mins / 60
    if lat_hemi == 'S':
        lat_dec = -1 * lat_dec

    # Longitude
    r_string = response_segments[0]
    lon_degs = int(r_string[0:3])
    lon_mins = float(r_string[3:-2])
    lon_hemi = r_string[-1]
    lon_dec = lon_degs + lon_mins / 60
    if lon_hemi == 'W':
        lon_dec = -1 * lon_dec

    # course
    r_string = response_segments[2]
    course = float(r_string)

    # speed
    r_string = response_segments[3]
    spd_knt = float(r_string)
    spd_kph = spd_knt * 1.852
    spd_mph = spd_knt * 1.151

    # altitude
    r_string = response_segments[4]
    altitude = float(r_string)

    # quality ?? is this fix status?
    r_string = response_segments[5]
    quality = int(r_string)

    # satellites
    r_string = response_segments[6]
    satellites = int(r_string)

    # hdop
    r_string = response_segments[7]
    hdop = float(r_string)

    # time
    r_string = response_segments[8]
    time_str = r_string.split(':')
    time_hrs = int(time_str[0])
    time_min = int(time_str[1])
    time_sec = int(time_str[2])

    # date
    r_string = response_segments[9]
    date_str = r_string.split('-')
    date_dd = int(date_str[0])
    date_mm = int(date_str[1])
    date_yr = int(date_str[2])

    print (lat_degs,lat_mins,lat_hemi,lat_dec)
    print (lon_degs,lon_mins,lon_hemi,lon_dec)
    print (time_hrs,time_min,time_sec,time_str)
    print (date_dd,date_mm,date_yr,date_str)
    print (spd_knt,spd_mph,spd_kph)
    print (altitude)
    print (course)
    print (hdop)
    print (satellites)
    print (quality)
else:
    print ('No response')

print ("Starting SD Card")
sd = SD()
os.mount(sd, '/sd')
# start new log file with headers
with open("/sd/log.csv", 'w') as Log_file:
    Log_file.write('remote_ID,GPSFix,latitude,longitude,voltage,rssi\n')

print ("Starting Webserver")
routes = WWW_routes()
mws = MicroWebSrv(routeHandlers=routes, webPath="/sd") # TCP port 80 and files in /sd
gc.collect()
mws.Start()         # Starts server in a new thread
gc.collect()

print ("Starting Lora")
lora = LoRa(mode=LoRa.LORA, region=LoRa.AU915)
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

print ('Starting MQTT')
mqtt = MQTTClient(my_ID, "io.adafruit.com",user="agmatthews", password="d9ee3d9d1d5a4f3b860d96beaa9d3413", port=1883)
mqtt.set_callback(mqtt_callback)
mqtt.connect()
mqtt.subscribe(topic="agmatthews/feeds/LORAtest")

print("testing CRC mod")
print (str(hex(crc16xmodem(b'I am a Walrus'))))

print ("Waiting for data")

while True:
    # feed the watch dog timer
    wdt.feed()
    # Free up memory by garbage collecting
    gc.collect()
    # if we havent had a fix recently then time out the most recent fix
    if time.time() - lastFix > FixTimeout:
        GPSFix = False
    # get some data from the LoRa buffer
    databytes = s.recv(256)
    stats = lora.stats()
    if len(databytes)>=40:
        GPSFix = False
        msgReceived = True
        if check_checksum(databytes):
            # unpack the data into seperate variables
            remote_ID, GPSFix, lat, lon, altitude, speed, course, vBatt, GPSdatetime, remoteMem, CRC = struct.unpack(dataStructure, databytes)
            # if this has Good GPS data then process it
            if GPSFix:
                # record the time of this fix in local seconds
                lastFix = time.time()
                # print received data to serial port / screen
                print(str(gc.mem_free()) + ',' + str(remoteMem) + ',RX:' + str(remote_ID) + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(vBatt) + ',' + str(GPSdatetime) + ',' + str(stats.rssi))
                # make a geoJSON package of the recived data
                geoJSON = {"geometry": {"type": "Point", "coordinates": [str(lon),str(lat)]}, "type": "Feature", "properties": {"remote_ID": str(remote_ID.decode()), "altitude": str(altitude), "speed": str(speed), "course": str(course), "battery": str(vBatt), "RSSI": str(stats.rssi), "datetime": str(GPSdatetime)}}
                # write received data to log file in CSV format in append mode
                with open("/sd/log.csv", 'a') as Log_file:
                    Log_file.write(remote_ID + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(vBatt) + ',' + str(stats.rssi) + '\n')
            else:
                # print received data to serial port / screen
                print(str(gc.mem_free()) + ',' + str(remoteMem) +',RX:' + str(remote_ID) + ",NOGPS," + str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(vBatt) + ',' + str(GPSdatetime) + ',' + str(stats.rssi))
        else:
            print ("Checksum ERROR")
            print ('Recv CRC: ')
            print (databytes[:-4])
            print ('Calc CRC: ')
            print (hex(calc_checksum(databytes)))

    time.sleep(receiveInterval)
