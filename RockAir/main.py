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
from tracker import tracker

# configuration
my_ID = 'BSE1' # unique id of this unit - 4 char string
receiveInterval = 2 # send data every 2 seconds
ledInterval = 1000 # update LED every 1000usec
WLAN_SSID = 'lerdy'
WLAN_PWD = 'lerdy0519'
dataStructure = '4sBffffffll1s' # structure packed data bytes received
dataSendInterval = 2 * 5 * 1000  # Send data to TracPlus every (2 * 60 * 1000)usec = 2 minutes
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
# periodically send data to MQTT server
# periodically send data to TracPlus via RockAir
    global GPSFix
    global dataSendInterval

    while True:
        if GPSFix > 0:
            # GPS OK so send a message
            print ("GPS FIX OK - so send a message")
            # Free up memory by garbage collecting
            gc.collect()
            # update location from tracker
            RockAir.getGPS()
            print (RockAir._latitude[3],RockAir._longitude[3])
            # send data to MQTT server
            mqtt.publish(topic="agmatthews/feeds/LORAtest", msg=remote_ID + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(GPSdatetime) + ',' + str(stats.rssi) + ',' + str(RockAir._latitude[3]) + ',' + str(RockAir._longitude[3]))

        #else:
            # GPS BAD so don't send message
            #
            #print ("NO GPS FIX - so don't send message")

        time.sleep_ms(int(dataSendInterval))

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

print ("Starting Serial Port")
uart1 = UART(1, 19200, bits=8, parity=None, stop=1)
uart1.init(baudrate=19200, bits=8, parity=None, stop=1)

print ("Starting Tracker")
RockAir = tracker(uart1, location_formatting='dd')
RockAir.getGPS()

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
                geoJSON = {"geometry": {"type": "Point", "coordinates": [str(lon),str(lat)]}, "type": "Feature", "properties": {"remote_ID": str(remote_ID.decode()), "altitude": str(altitude), "speed": str(speed), "course": str(course), "battery": str(vBatt), "RSSI": str(stats.rssi), "datetime": str(GPSdatetime)}} #, "RockAir_Lat": str(lat_dec), "RockAir_Lon": str(lon_dec), "RockAir_Time": time_str}}
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
