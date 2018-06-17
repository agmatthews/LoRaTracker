import uctypes

def check_checksum( databytes ):
    """
    Check the checksum on a databytes object.
    Skips everything before first $ and stops at first *
    calculates the basic XOR checksum and compares it to the stored one
"""
    # define the start and end characters for each message
    startchar = b'$'
    endchar = b'*'

    # Find the start and end of the data sentence
    startIndex = databytes.find(startchar)
    endIndex = databytes.find(endchar)

    # if we cant find the start and end chars then bail out
    if startIndex<0 or endIndex<=startIndex:
        return False

    # iterate through the bytes calculating the checksum
    calcCRC = 0
    for c in databytes[startIndex:endIndex]:
        calcCRC ^= c
    calcCRC = calcCRC & 0xFF

    # Find the checksum at the end of the string
    storedCRC = databytes[endIndex+1:endIndex+5]

    # convert the calculated and stored CRC's to strings for comparing
    calcCRCStr = str(b""+hex(calcCRC))
    storedCRCStr = str(storedCRC)

    # if they are the same then we are good
    if calcCRCStr == storedCRCStr:
        return True

    return False


def calc_checksum( databytes ):
    """
    calculate a basic XOR checksum on a databytes object.
    Skips everything before first $ and stops at first *
    """

    # define the start and end characters for each message
    startchar = b'$'
    endchar = b'*'

    # Find the start and end of the data sentence
    startIndex = databytes.find(startchar)
    endIndex = databytes.find(endchar)

    # if we cant find the start and end chars then bail out
    if startIndex<0 or endIndex<=startIndex:
        return -1

    # iterate through the bytes calculating the checksum
    result = 0
    for c in databytes[startIndex:endIndex]:
        result ^= c
    result = result & 0xFF

    return result
