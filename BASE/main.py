import pycom
import machine
from network import LoRa
from network import WLAN
import socket
from microWebSrv import MicroWebSrv
from mqtt import MQTTClient
import time
import struct
import gc
import ujson
from machine import SD
import os

# configuration
my_ID = 'BSE1' # unique id of this unit - 4 char string
ledInterval = 1000 # update LED every 1000usec
WLAN_SSID = 'lerdy'
WLAN_PWD = 'lerdy0519'
dataStructure = '4sBffffffl' # structure for packing data into bytes to send

GPSFix = False
GPSdatetime = None
lat = None
lon = None
altitude  = None
speed = None
course = None
remote_ID = b''
vBatt = 0.0

def sub_cb(topic, msg):
   print(msg)

print ('Starting A-BASE')
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

print ("Starting SD Card")
sd = SD()
os.mount(sd, '/sd')
# start new log file with headers
with open("/sd/log.csv", 'w') as Log_file:
    Log_file.write('remote_ID,GPSFix,latitude,longitude,rssi\n')

print ("Starting Webserver")
mws = MicroWebSrv(webPath="/sd") # TCP port 80 and files in /sd
gc.collect()
mws.Start()         # Starts server in a new thread
gc.collect()

print ("Starting Lora")
lora = LoRa(mode=LoRa.LORA, region=LoRa.AU915)
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

print ('Starting MQTT')
mqtt = MQTTClient(my_ID, "io.adafruit.com",user="agmatthews", password="d9ee3d9d1d5a4f3b860d96beaa9d3413", port=1883)
mqtt.set_callback(sub_cb)
mqtt.connect()
mqtt.subscribe(topic="agmatthews/feeds/LORAtest")

print ("Waiting for data")

while True:
    databytes = s.recv(256)
    stats = lora.stats()
    GPSFix = False
    if len(databytes)>4:
        remote_ID, GPSFix, lat, lon, altitude, speed, course, vBatt, GPSdatetime = struct.unpack(dataStructure, databytes)
        if GPSFix:
            print(remote_ID + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + ',' + str(GPSdatetime) + ',' + str(stats.rssi))
            geoJSON = {"geometry": {"type": "Point", "coordinates": [str(lon),str(lat)]}, "type": "Feature", "properties": {}}
            with open("/sd/log.csv", 'a') as Log_file:
                Log_file.write(remote_ID + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(stats.rssi) + '\n')
            with open("/sd/gps.json", 'w') as JSON_file:
                JSON_file.write(ujson.dumps(geoJSON))
            #mqtt.publish(topic="agmatthews/feeds/LORAtest", msg=remote_ID + ',' + str(GPSFix) + ',' + str(lat) + ',' + str(lon) + ',' + str(GPSdatetime) + ',' + str(stats.rssi))
            pycom.rgbled(0x001100)
        else:
            print(remote_ID + ",NOGPS," + str(lat) + ',' + str(lon) + ',' + str(altitude) + ',' + str(speed) + ',' + str(course) + str(GPSdatetime) + ',' + str(stats.rssi))
            pycom.rgbled(0x110000)
    time.sleep(1.5)
    pycom.rgbled(0x000000)
    time.sleep(0.5)
