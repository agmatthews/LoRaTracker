#
import pycom
import machine
import time

def RockAir_getGPS(uart1):
    """
    do stuff
    """
    # request RockAir GPS data on serial port
    uart1.write('R7+GPS\r')
    # wait for a second
    time.sleep(1)
    # get response from RockAir
    RockAirResponse = b''
    # while there is data on the serial port process it
    while uart1.any():
        # read one line from serial port
        RockAirResponse = uart1.readline()
        # if this line is long enough to be the GPS response then processes it
        if len(RockAirResponse)>5:
            #turn response into string
            RockAirResponse = RockAirResponse.decode()

            # strip off any carridge returns or new line characters
            RockAirResponse = RockAirResponse.replace('\r', '')
            RockAirResponse = RockAirResponse.replace('\n', '')

            # split the response into segments at each comma
            response_segments = str(RockAirResponse).split(',')

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

            # debug printing
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

            # once the response is processed the break the loop and return
            break

    return
