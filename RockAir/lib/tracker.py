import pycom
import machine
import time
import ubinascii
from math import floor, modf
from machine import UART
import crc16

rockAirBaud = 19200 # baud rate for rock air tracker
rockAirTx = 'P20' # this TX pin on the pycom connects to T1in on the max232 - blue
rockAirRx = 'P21' # this RX pin on the pycom connects to R1out on the max232 - yellow

class tracker(object):
    """
    RockAir Sentence Parser.
    Creates object that stores all relevant tracker data and statistics.
    #Parses sentences one line at a time using update().
    """
    __MONTHS = ('January', 'February', 'March', 'April', 'May',
                'June', 'July', 'August', 'September', 'October',
                'November', 'December')

    def __init__(self, local_offset=0, location_formatting='ddm'):
        """
        Setup GPS Object Status Flags, Internal Data Registers, etc
            local_offset (int): Timzone Difference to UTC
            location_formatting (str): Style For Presenting Longitude/Latitude:
                                       Decimal Degree Minute (ddm) - 40° 26.767′ N
                                       Degrees Minutes Seconds (dms) - 40° 26′ 46″ N
                                       Decimal Degrees (dd) - 40.446° N
        """

        #####################
        # serial port
#Testing
        uart = UART(1, baudrate=rockAirBaud, bits=8, parity=None, stop=1, pins=(rockAirTx,rockAirRx))
        self.uart = uart

        #####################
        # Object Status Flags
        self.sentence_active = False
        self.active_segment = 0
        self.process_crc = False
        self.gps_segments = []
        self.crc_xor = 0
        self.char_count = 0
        self.fix_time = 0

        #####################
        # Sentence Statistics
        self.crc_fails = 0
        self.clean_sentences = 0
        self.parsed_sentences = 0

        #####################
        # Logging Related
        self.log_handle = None
        self.log_en = False

        #####################
        # Data From Sentences
        # Time
        self.timestamp = (0, 0, 0)
        self.date = (0, 0, 0)
        self.local_offset = local_offset

        # Position/Motion
        self._latitude = (0, 0.0, 'N', 0.0)
        self._longitude = (0, 0.0, 'W', 0.0)
        self.coord_format = location_formatting
        self.speed = (0.0, 0.0, 0.0)
        self.course = 0.0
        self.altitude = 0.0
        self.geoid_height = 0.0

        # GPS Info
        self.satellites_in_view = 0
        self.satellites_in_use = 0
        self.satellites_used = []
        self.last_sv_sentence = 0
        self.total_sv_sentences = 0
        self.satellite_data = dict()
        self.hdop = 0.0
        self.pdop = 0.0
        self.vdop = 0.0
        self.valid = False
        self.fix_stat = 0
        self.fix_type = 1

    ########################################
    # Coordinates Translation Functions
    ########################################
    @property
    def lat(self):
        """return decimal_degrees latitude"""
        return self._latitude[3]
    @property
    def lon(self):
        """return decimal_degrees longitude"""
        return self._longitude[3]

    @property
    def latitude(self):
        """Format Latitude Data Correctly"""
        if self.coord_format == 'dd':
            decimal_degrees = self._latitude[0] + (self._latitude[1] / 60)
            return [decimal_degrees, self._latitude[2]]
        elif self.coord_format == 'dms':
            minute_parts = modf(self._latitude[1])
            seconds = round(minute_parts[0] * 60)
            return [self._latitude[0], int(minute_parts[1]), seconds, self._latitude[2]]
        else:
            return self._latitude

    @property
    def longitude(self):
        """Format Longitude Data Correctly"""
        if self.coord_format == 'dd':
            decimal_degrees = self._longitude[0] + (self._longitude[1] / 60)
            return [decimal_degrees, self._longitude[2]]
        elif self.coord_format == 'dms':
            minute_parts = modf(self._longitude[1])
            seconds = round(minute_parts[0] * 60)
            return [self._longitude[0], int(minute_parts[1]), seconds, self._longitude[2]]
        else:
            return self._longitude


    ########################################
    # Sentence Parsers
    ########################################


    def getGPS(self):
        """
        #
        #
        """
        self.valid = False
        # clear any unread chars from serial bus
        if self.uart.any():
            self.uart.readall()
        # request RockAir GPS data on serial port
        self.uart.write('R7+GPS\r')
        # wait for a second
        time.sleep(1)
        # get response from RockAir
        response = b''
        # while there is data on the serial port process it
        while self.uart.any():
            # read one line from serial port
            response = self.uart.readline()
            # if this line is long enough to be the GPS response then processes it
            if len(response)>5:
                #turn response into string
                response = response.decode()
                # strip off any carridge returns or new line characters
                response = response.replace('\r', '')
                response = response.replace('\n', '')
                # split the response into segments at each comma
                response_segments = str(response).split(',')
                try:
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
                    # multiply altitude by factor of 10
                    altitude = altitude / 10.0
                    # quality ?? is this fix status?
                    r_string = response_segments[5]
                    quality = int(r_string)
                    # satellites
                    r_string = response_segments[6]
                    sats_used = int(r_string)
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
                    # Update Object Data
                    self._latitude = (lat_degs, lat_mins, lat_hemi, lat_dec)
                    self._longitude = (lon_degs, lon_mins, lon_hemi, lon_dec)
                    self.speed = (spd_knt, spd_mph, spd_kph)
                    self.course = course
                    self.altitude = altitude
                    self.satellites_used = sats_used
                    self.hdop = hdop
                    self.timestamp = (time_hrs, time_min, time_sec)
                    self.date = (date_dd, date_mm, date_yr)
                    self.valid = True
                    # once the response is processed clear the uart and break the loop to return
                    if self.uart.any():
                        self.uart.readall()
                    break
                except ValueError:  # Bad Data
                    self.valid = False
                    print('Tracker getGPS value ERROR')
                    print(response_segments)
                    return
                except IndexError:  # Bad Data
                    self.valid = False
                    print('Tracker getGPS index ERROR')
                    print(response_segments)
                    return
        return

    def sendMessage(self, message):
        """
        #
        #
        """
## not working second time we come here - WHY?
# temp fix by deinit and re init the uart
#        self.uart.deinit()
#        self.uart.init(baudrate=19200, bits=8, parity=None, stop=1)
        # wait for a second
#        time.sleep(1)
        # clear any unread chars from serial bus
        if self.uart.any():
            self.uart.readall()
        # build up message to send from components
        messageCMD = 'R7+WHP='
        messageTYP = '00'
        # leading zeros do not change crc calc no need to include them for message type 0x00
        messageCRC = hex(crc16.xmodem(message.encode()))[2:].upper()
        messageHex = ubinascii.hexlify(message).decode().upper()
        messageEnd = '\r'
        messageData = messageCMD + messageCRC + messageTYP + messageHex + messageEnd

        print('   Sending : ' + message)
        print('   Length  : ' + str(len(messageData)))

        # request RockAir send a message with position
        self.uart.write(messageData)
        # wait for a second
        time.sleep(1)
        # get response from RockAir
        response = b''
        responseTxt = ''
        responseNum = 0

        if self.uart.any():
            # first line is just \n - read it and move on
            response = self.uart.readline()

        if self.uart.any():
            # second line is message number - read it and store it
            response = self.uart.readline()

            #turn response into string
            response = response.decode()

            # strip off any carrige returns or new line characters
            response = response.replace('\r', '')
            response = response.replace('\n', '')

            try:
                responseNum = int(response)
            except (ValueError, TypeError):
                responseNum = 0

        if self.uart.any():
            # third line is RockAir response text - read it and store it
            response = self.uart.readline()

            #turn response into string
            response = response.decode()

            # strip off any carrige returns or new line characters
            response = response.replace('\r', '')
            response = response.replace('\n', '')

            responseTxt = response

        if self.uart.any():
            # forth line is just \r - read it and move on
            response = self.uart.readline()

            # clear the UART of any remaiing data
            if self.uart.any():
                self.uart.readall()

            print ('   Response: ' + responseTxt)
            print ('   Message : ' + str(responseNum))

        else:
            print ('   Sending ERROR')

        return responseNum
