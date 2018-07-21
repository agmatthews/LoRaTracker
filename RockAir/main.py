import pycom
import machine
from network import LoRa
from network import WLAN
from machine import SD
from machine import WDT
from machine import UART
from microWebSrv import MicroWebSrv
from robust import MQTTClient
import socket
import time
import utime
import _thread
import struct
import gc
import ujson
import os
import ubinascii
import uctypes
import crc16
from tracker import tracker
from rgb import RGBLED
from geometry import gDistance, gBearing

##################################################
## initial configuration
##################################################
config = {
    'WDtimeout': int(25 * 1000), # use watchdog timer to reset the board if it does not update reguarly (25 seconds)
    'FixTimeout': 1000 * 30,  # 30 seconds in ms
    'receiveInterval': 2, # recive data every 2 seconds
    'trackerSendInterval': 1 * 60 * 1000,  # Send data to TracPlus every (2 * 60 * 1000)usec = 2 minutes
    'mqttSendInterval': 1 * 30 * 1000,  # Send data to TracPlus every (1* 30 * 1000)usec = 30 seconds
    }

##################################################
## Variables
##################################################

# instantiate libraries
wdt = WDT(timeout=config['WDtimeout']) # enable a Watchdog timer with a specified timeou
led = RGBLED(10)  # start the status LED library
rtc = machine.RTC() # start the real time clock library

# initialise variables
msgReceived = False
GPSFix = False
lastFix = utime.time() - config['FixTimeout']
last_recv_time = utime.time() - config['receiveInterval']
last_msg_send = utime.time() - config['trackerSendInterval']
last_mqtt_send = utime.time() - config['mqttSendInterval']
geoJSON = {}
status = {}
status["loraStats"] = {}
status["deltaLat"] = 0
status["deltaLon"] = 0
msgCount = 0
crcErrorCount = 0
last_LED_check = 0
LED_count = 0
network_OK = False
internet_OK = False
tracker_OK = False

##################################################
## Functions
##################################################

def update_LED():
# continuously update the status of the unit via the onboard LED
    global config
    global GPSFix
    global msgReceived
    global last_LED_check
    global LED_count
    ledColour = 0x000000

    if(utime.ticks_ms() - last_LED_check > config['ledInterval']/100):
        last_LED_check = utime.ticks_ms()
        LED_count += 1
        if (LED_count<90):
            if GPSFix:
                # GPS OK so set LED to Green
                ledHue= led.GREEN
            else:
                # GPS BAD so set LED to RED
                ledHue= led.RED
            if GPSFix and utime.time() - lastFix > config['staleGPStime']:
                # No GPS fix for staleGPStime seconds so set LED to Orange
                ledHue= led.ORANGE
            if msgReceived == True:
                # Just sent a message so set the LED to Magenta
                ledHue= led.MAGENTA
                #reset the msg flag as we have actioned it
                msgReceived = False
            # set the LED colour with variable lightness based on LED_count
            led.hl(ledHue,LED_count)
        else:
            # blink LED off for 10% of the time
            led.off()
            LED_count = 0

def sendMqttMsg():
# periodically send data to MQTT server
    global config
    global GPSFix
    global rxData
    global last_mqtt_send

    if(utime.ticks_ms() - last_mqtt_send > config['mqttSendInterval']):
        last_mqtt_send = utime.ticks_ms()
        if GPSFix > 0:
            # GPS OK so send a message
            print ("GPS OK - send MQTT message")
            # Free up memory by garbage collecting
            gc.collect()
            try:
                # create message to send
                theMsg = rxData["uid"] + ',' + str(rxData["fix"]) + ',' + str(rxData["lat"]) + ',' + str(rxData["lon"]) + ',' + str(rxData["gdt"]) + ',' + str(status["loraStats"].rssi) + ',' + str(RockAir._latitude[3]) + ',' + str(RockAir._longitude[3])
                # send data to MQTT server
                mqtt.publish(topic=config['MQTTtopic'], msg=theMsg)
            except Exception as e:
                print('   Data Error - No send via MQTT')
                print(rxData)
        else:
            print('No FIX - No send via MQTT')

def mqtt_callback(topic, msg):
    # this subroutine would process any message that comes back from MQTT server
    print(msg)

def sendTrackerMsg():
# periodically send data to TracPlus via RockAir
    global GPSFix
    global rxData
    global config
    global last_msg_send
    global RockAir

    if(utime.ticks_ms() - last_msg_send > config['trackerSendInterval']):
        last_msg_send = utime.ticks_ms()
        if GPSFix > 0:
            # GPS OK so send a message
            print ("GPS OK - send Tracker message")
            # Free up memory by garbage collecting
            gc.collect()
            if (RockAir.valid):
                tracker_OK = True
                print('   Tracker OK: ',RockAir._latitude[3],RockAir._longitude[3])
                # create a dictionary object to hold remote message data
                try:
                    messageData = {}
                    messageData["EVT"] = 'PROXD'
                    messageData["ID"] = rxData["uid"]
                    messageData["LATD"] = status["deltaLat"] #lat delta
                    messageData["LOND"] = status["deltaLon"] #lon delta
                    messageData["SPD"] = "{:.0f}".format(rxData["spd"])
                    messageData["COG"] = "{:.0f}".format(rxData["cog"])
                    messageData["ALT"] = "{:.0f}".format(rxData["alt"])
                    messageData["BAT"] = "{:.2f}".format(rxData["bat"])
                    # encode the message data as JSON without wasted spaces
                    encodedData = ujson.dumps(messageData).replace(" ", "")
                    # send the remote message
                    RockAir.sendMessage(encodedData)
                except Exception as e:
                    print('   Data Error - No send via Tracker')
                    print(rxData)
            else:
                print('   No valid Tracker')
        else:
            print('No FIX - No send via Tracker')

def WWW_routes():
    global geoJSON, statusJSON,configJSON
    def _geojson(client, response):
        response.WriteResponseJSONOk(geoJSON)
    def _statusjson(client, response):
        response.WriteResponseJSONOk(status)
    def _configjson(client, response):
        response.WriteResponseJSONOk(config)
    def _configure(client, response, arguments):
        print ('Got config '+ str(arguments['item']) + ':' + str(arguments['value']))
        if arguments['item'] in config:
            config[arguments['item']] = arguments['value']
            print ('Updated : ' + arguments['item'])
            print ('Value   : ' + config[arguments['item']])
        response.WriteResponseJSONOk(config)
    def _sendtxt(client, response, arguments):
        msgTxt = ""
        if arguments['item'] == 'msg':
            msgTxt = str(arguments['value'])
            msgTxt = msgTxt.replace(" ", "_")
            msgTxt = msgTxt.replace("%20", "_")
            # Free up memory by garbage collecting
            gc.collect()
            try:
                result = RockAir.sendMessage(msgTxt)
                if result:
                    response.WriteResponseJSONOk("{'result':'OK', 'msg':'" + msgTxt + "'}")
                else:
                    response.WriteResponseJSONOk("{'result':'ERROR'}")
            except Exception as e:
                print('   Data Error - No text via Tracker')
                response.WriteResponseJSONOk("{'result':'ERROR'}")

    return [('/gps.json', 'GET', _geojson),('/status.json', 'GET', _statusjson),('/config.json', 'GET', _configjson), ('/config/<item>/<value>', 'GET', _configure), ('/sendtxt/<item>/<value>', 'GET', _sendtxt)]

##################################################
## MAIN loop
##################################################

# print software version info
print (os.uname())

print ('Starting BASE (LoRaTracker)')

print ("Starting SD Card")
sd = SD()
os.mount(sd, '/sd')
# read config
print ('   Reading config')
with open('config.txt', 'r') as configFile:
    configText = configFile.read()
    config = eval(configText)
# Start logfile
print ('   Starting logfile')
maxIndex = 0
# loop through all the files on the SD card
for f in os.listdir('/sd/logs'):
    #look for 'LogFileBase'nnnn files
    fnLen = len(config['LogFileBase'])
    if f[:fnLen]==config['LogFileBase']:
        try:
            # extract the number from the GPSlognnnn filename
            index = int(f[fnLen:].split(".")[0])
        except ValueError:
            index = 0
        # if this is the highest numbered file then record it
        if index > maxIndex:
            maxIndex = index
if maxIndex>9999:
    print ('   SD card file name error - too many files')
# create a new filename one number higher that the highest on theSD card
log_filename = '/sd/logs/' + config['LogFileBase'] + '{:04d}.csv'.format(maxIndex+1)
config['log_filename'] = log_filename
print('   Logfile: ' + log_filename)
# start new log file with headers
with open(log_filename, 'a') as Log_file:
    Log_file.write(str(rtc.now()) + '\n')
    Log_file.write('remote_ID,GPSFix,latitude,longitude,voltage,rssi\n')

print ('UnitID: ' + str(config['my_ID']))

print ("Starting LED")
led.h(led.BLUE)

print ("Starting Network")
wlan = WLAN()
wlan.mode(WLAN.STA)
# scan for available networks
available_nets = wlan.scan()
for net in available_nets:
    print('   Found SSID: '+ net.ssid)
nets = frozenset([e.ssid for e in available_nets])
# match available nets with known nets
known_nets_names = frozenset([key for key in config['known_nets']])
net_to_use = list(nets & known_nets_names)
# try and use the first matching network
try:
    net_to_use = net_to_use[0]
    net_properties = config['known_nets'][net_to_use]
    pwd = net_properties['pwd']
    sec = [e.sec for e in available_nets if e.ssid == net_to_use][0]
    print('   Connecting to: ' + net_to_use)
    if 'wlan_config' in net_properties:
        wlan.ifconfig(config=net_properties['wlan_config'])
    wlan.connect(net_to_use, auth=(sec, pwd), timeout=5000)
    while not wlan.isconnected():
        # idle power while waiting
        machine.idle()
    print('   Connected.')
    print('   IP address: ' + wlan.ifconfig()[0])
    network_OK = True
    internet_OK = True

except Exception as e:
    print('   Cant connect to known networks')
    print('   Entering AP mode')
    wlan.init(mode=WLAN.AP, ssid=config['SSID'], channel=6, antenna=WLAN.INT_ANT)
    network_OK = True

print('Starting Clocks')
if network_OK:
    print('   Syncing RTC to '+ config['ntp_source'])
    rtc.ntp_sync(config['ntp_source'])
    utime.sleep_ms(1500)
    print('   RTC Time :', rtc.now())
utime.timezone(config['TZ_offset_secs'])
print('   Local Time:', utime.localtime())


print ("Starting Tracker")
print ("   Open Serial Port")
RockAir = tracker(location_formatting='dd')
# get the current location from the tracker
RockAir.getGPS()
if (RockAir.valid):
# check we got some data
    tracker_OK = True
    print('   Tracker OK: ',RockAir._latitude[3],RockAir._longitude[3])
    # create a dictionary object to hold startup message data
    startupData = {}
    startupData["EVT"] = 'STARTUP'
    startupData["TYP"] = 'PROX_RX'
    startupData["ID"] = config['my_ID']
    startupData["SW"] = config['swVer']
    startupData["HW"] = config['hwVer']
    # encode the message data as JSON without spaces
    encodedData = ujson.dumps(startupData).replace(" ", "")
    # send the startup message
    RockAir.sendMessage(encodedData)
else:
    print('   Tracker ERROR')
    tracker_OK = False

if config['use_WebServer'] and network_OK:
    print ("Starting Webserver")
    routes = WWW_routes()
    mws = MicroWebSrv(routeHandlers=routes, webPath="/sd") # TCP port 80 and files in /sd
    gc.collect()
    mws.Start()
    gc.collect()

if config['use_MQTT'] and internet_OK:
    print ('Starting MQTT')
# put details in config
    mqtt = MQTTClient(config['my_ID'], config['MQTTserver'],user=config['MQTTuser'], password=config['MQTTpass'], port=config['MQTTport'])
    mqtt.set_callback(mqtt_callback)
    mqtt.connect()
    mqtt.subscribe(topic=config['MQTTtopic'])

print ("Starting Lora")
lora = LoRa(mode=LoRa.LORA, region=LoRa.AU915)
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)

print ("Waiting for data")

while True:
    # feed the watch dog timer
    wdt.feed()
    # update the status LED
    update_LED()
    # Free up memory by garbage collecting
    gc.collect()
    # periodically send message via Tracker
    if config['use_Tracker'] and tracker_OK:
        sendTrackerMsg()
    # periodically send message via MQTT
    if config['use_MQTT'] and internet_OK:
        sendMqttMsg()
    # if we havent had a fix recently then time out the most recent fix
    if utime.time() - lastFix > config['FixTimeout']:
        GPSFix = False
    # if it is time to check for a message then check for it
    if(utime.time() > last_recv_time + config['receiveInterval']):
        # get some data from the LoRa buffer
        databytes = s.recv(256)
        status["loraStats"] = lora.stats()
        status["timeNow"] = rtc.now()
        if len(databytes)>=40:
            print (' ')
            print ("Message Received")
            # record the time of this fix in local seconds
            last_recv_time = utime.time()
            rxData = {}
            rxData["fix"] = False
            msgReceived = True
            msgCount += 1
            status["msgCount"] = msgCount
            # check the crc on the recived message
            if crc16.checkcrc(databytes):
                # CRC is OK  - process message
                rxData = ujson.loads(databytes[6:].decode())
                # print received data to debug
                print(rxData)
                status["rxData"] = rxData
                #print(status["loraStats"].rssi)
                # check GPS data in the recived data
                if rxData["fix"] and rxData["lon"]!=0:
                    # GPS is good - process message
                    GPSFix = True
                    # record the time of this fix in local seconds
                    lastFix = utime.time()
                    # calculate delta between Base and remote node
                    RockAir.getGPS()
                    if (RockAir.valid):
                        status["RockAirValid"] = RockAir.valid
                        status["RockAirLat"] = RockAir.lat
                        status["RockAirLon"] = RockAir.lon
                        status["RockAirTime"] = RockAir.timestamp
# add more rockair items here eg speed alt etc
                        status["deltaLat"] = int((RockAir.lat - rxData["lat"])*100000)
                        status["deltaLon"] = int((RockAir.lon - rxData["lon"])*100000)
                        status["bearing"] = gBearing(RockAir.lon, RockAir.lat, rxData["lon"], rxData["lat"])
                        status["distance"] = gDistance(RockAir.lon, RockAir.lat, rxData["lon"], rxData["lat"])*1000
                        #print('Remote:   Bearing: ' + str(int(status["bearing"])) + ' Distance: ' + str(int(status["distance"])))
                    else:
                        print('No valid Tracker ERROR')
                        status["deltaLat"] = 0
                        status["deltaLon"] = 0
                        status["bearing"] = 0
                        status["distance"] = 0
                    #print(status)
                    # make a geoJSON package of the recived data
                    geoJSON = { "type": "FeatureCollection",
                                "features":
                                [
                                    {
                                        "type": "Feature",
                                        "properties": {
                                            "id": rxData["uid"],
                                            "type": 'drone',
                                            "altitude": str(rxData["alt"]),
                                            "speed": str(rxData["spd"]),
                                            "course": str(rxData["cog"]),
                                            "battery": str(rxData["bat"]),
                                            "datetime": str(rxData["gdt"])
                                        },
                                        "geometry": {
                                            "type": "Point",
                                            "coordinates": [rxData["lon"],rxData["lat"]]
                                        }
                                    },
                                    {
                                        "type": "Feature",
                                        "properties": {
                                            "id": config['my_ID'],
                                            "type": 'base',
                                            "RSSI": str(status["loraStats"].rssi),
                                            "RockAir_Lat": str(RockAir.lat),
                                            "RockAir_Lon": str(RockAir.lon),
                                            "datetime": str(RockAir.timestamp)
                                        },
                                        "geometry": {
                                            "type": "Point",
                                            "coordinates": [RockAir.lon,RockAir.lat]
                                        }
                                    },
                                    {
                                        "type": "Feature",
                                        "properties": {
                                            "id": "line",
                                            "name": "linus"
                                        },
                                        "geometry": {
                                            "type": "LineString",
                                            "coordinates": [[RockAir.lon, RockAir.lat], [rxData["lon"], rxData["lat"]]]
                                        }
                                    }
                                ]
                            }
                    # write received data to log file in CSV format in append mode
                    with open(log_filename, 'a') as Log_file:
                        Log_file.write(str(status["timeNow"]))
                        try:
                            # create message to send
                            theMsg = rxData["uid"] + ',' + str(rxData["fix"]) + ',' + str(rxData["lat"]) + ',' + str(rxData["lon"]) + ',' + str(rxData["gdt"]) + ',' + str(status["loraStats"].rssi) + ',' + str(RockAir._latitude[3]) + ',' + str(RockAir._longitude[3])
                            # write data to SD Card
                            Log_file.write(rxData["uid"] + ',' + str(rxData["fix"]) + ',' + str(rxData["lat"]) + ',' + str(rxData["lon"]) + ',' + str(rxData["bat"]) + ',' + str(status["loraStats"].rssi) + '\n')
                        except Exception as e:
                            print('   Data Error - No SD card write')
                            print(rxData)
                else:
                    print ("GPS BAD")
            else:
                crcErrorCount += 1
                status["crcErrorCount"] = crcErrorCount
                print ('ERROR - Checksum.')
                print ('  Messages: ' + str(msgCount) + ' recived, ' + str(crcErrorCount) + ' bad' )
                print ('  Recv CRC: ' + str(databytes[:6].decode()))
                print ('  Calc CRC: ' + str(hex(crc16.xmodem(databytes[6:]))))
                print ('  Rcv data: ' + str(databytes.decode()))
    utime.sleep_ms(config['napTime'])
