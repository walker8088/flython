
import math

def twos_compliment(high_byte, low_byte):
    value = (high_byte << 8) + low_byte
    if (value >= 0x8000):
        return -((0xffff - value) + 1)
    else:
        return value

def bytes_to_int(msb, lsb):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return -(((msb ^ 255) << 8) | (lsb ^ 255) + 1)

def get_x_rotation(x,y,z):
    radians = math.atan(x / math.sqrt(y * y + z * z))
    return math.degrees(radians)

def get_y_rotation(x,y,z):
    radians = math.atan(y / math.sqrt(x * x, z * z))
    return math.degrees(radians)
