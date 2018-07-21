import pycom
import machine
from network import LoRa
from network import WLAN
from machine import SD
from machine import WDT
from machine import UART
from machine import Pin
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
from mfrc522 import MFRC522

##################################################
## initial configuration
##################################################
config = {
    'WDtimeout': int(25 * 1000), # use watchdog timer to reset the board if it does not update reguarly (25 seconds)
    'use_Tracker': True, # if true connect to the RockAir Tracker and get position and send data
    'use_WebServer': True, # if true and if network available fire up the local web server
    'use_MQTT': True, # if true and if internet available send data to MQTT server
    'known_nets': { 'Galilean': {'pwd': 'ijemedoo'}, 'lerdy': {'pwd': 'lerdy0519'}, 'iPhone': {'pwd': 'Passw0rd'} }, # known WLAN networks to attempt connection to
    'my_ID': 'CARD', # unique id of this unit - 4 char string
    'TZ_offset_secs': 10 * 60 * 60,  # AEST is +10 hours offset from UTC
    'ntp_source': 'pool.ntp.org', # URL for network time protocol source
    'ledInterval': 1000, # update LED every 1000msec
    'ledLightness': 5, # brightness value for indicator LED
    'napTime': 2500, # number of milli seconds to nap to allow other things to happening
    'cardInterval': 5000 # munber of milliseconds between card reads for same card
    }

##################################################
## Variables
##################################################

# instantiate libraries
wdt = WDT(timeout=config['WDtimeout']) # enable a Watchdog timer with a specified timeou
led = RGBLED(10)  # start the status LED library
rtc = machine.RTC() # start the real time clock library
#rdr = MFRC522("GP14", "GP16", "GP15", "GP22", "GP17")

# initialise variables
status = {}
tagRx = 0
last_LED_check = 0
LED_count = 0
network_OK = False
internet_OK = False
blinkState = 0
cardDetected = False
lastCard = []
lastCardTime = utime.ticks_ms() - config['cardInterval']

##################################################
## Functions
##################################################

def update_LED():
# continuously update the status of the unit via the onboard LED
    global config
    global cardDetected
    global blinkState
    global last_LED_check
    global LED_count
    ledHue= led.BLUE

    if(utime.ticks_ms() - last_LED_check > config['ledInterval']/100):
        last_LED_check = utime.ticks_ms()
        LED_count += 1
        if (LED_count<90):
            if blinkState:
                # GPS OK so set LED to Green
                ledHue= led.GREEN
            else:
                # GPS BAD so set LED to RED
                ledHue= led.RED
            if cardDetected == True:
                # Just sent a message so set the LED to Magenta
                ledHue= led.MAGENTA
                #reset the msg flag as we have actioned it
                cardDetected = False
            # set the LED colour with variable lightness based on LED_count
            led.hl(ledHue,LED_count)
        else:
            # blink LED off for 10% of the time
            led.off()
            LED_count = 0

def card_read():
    global cardDetected
    global lastCard

    rdr = MFRC522(Pin.exp_board.G14, Pin.exp_board.G16, Pin.exp_board.G15, Pin.exp_board.G22, Pin.exp_board.G17)

    while True:
        (stat, tag_type) = rdr.request(rdr.REQIDL)
        if stat == rdr.OK:
            (stat, raw_uid) = rdr.anticoll()
            if stat == rdr.OK:
                if lastCard != raw_uid or utime.ticks_ms() - lastCardTime > config['cardInterval']:
                    lastCardTime = utime.ticks_ms()
                    lastCard = raw_uid
                    cardDetected = True
                    #print("New card detected")
                    #print("  - tag type: 0x%02x" % tag_type)
                    #print("  - uid	 : 0x%02x%02x%02x%02x" % (raw_uid[0], raw_uid[1], raw_uid[2], raw_uid[3]))
                    if rdr.select_tag(raw_uid) == rdr.OK:
                        key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
                        if rdr.auth(rdr.AUTHENT1A, 8, key, raw_uid) == rdr.OK:
                            #print("Address 8 data: %s" % rdr.read(8))
                            rdr.stop_crypto1()
                        else:
                            print("Authentication error")
                    else:
                        print("Failed to select tag")

##################################################
## MAIN loop
##################################################

# print software version info
print (os.uname())

print ('Starting BASE (RC522 test)')

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


print ("Starting Card Reader")
_thread.start_new_thread(card_read, ())

while True:
    # feed the watch dog timer
    wdt.feed()
    # Free up memory by garbage collecting
    gc.collect()
    #
    if(cardDetected):
        print (str(ubinascii.hexlify(bytearray(lastCard))))
        cardDetected = False
    # update the status LED
    update_LED()
    blinkState = not blinkState
    #
    utime.sleep_ms(config['napTime'])
