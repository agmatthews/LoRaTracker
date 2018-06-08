
def check_checksum( databytes ):
    """
    Check the checksum of databytes sentence.
    Skips anything before $ and stops at first *
    """

    # Find the start and end of the data sentence
    startchar = b'$'
    endchar = b'*'

    startIndex = databytes.find(startchar)
    endIndex = databytes.find(endchar)
    if startIndex<0 or endIndex<0:
        return (False, None, None)

    # iterate through the bytes calculating the checksum
    result = 0
    for c in databytes[startIndex:endIndex]:
        result ^= c
    result = result & 0xFF
    resultStr = str(b""+hex(result))

    # Find the checksum at the end of the string
    crc = databytes[endIndex+1:endIndex+5]
    crcStr = str(crc)

    if resultStr == crcStr:
        return (True, resultStr, crcStr)

    return (False, resultStr, crcStr)


def calc_checksum( databytes ):
    """
    calculate a checksum on a databytes object.
    Skips before $ and stops at first *
    """

    # Find the start and end of the data sentence
    startchar = b'$'
    endchar = b'*'

    startIndex = databytes.find(startchar)
    endIndex = databytes.find(endchar)
    if startIndex<0 or endIndex<0:
        return (False, None, None)

    # iterate through the bytes calculating the checksum
    result = 0
    for c in databytes[startIndex:endIndex]:
        result ^= c
    result = result & 0xFF

    return result
