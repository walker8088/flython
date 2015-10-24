
import smbus

# ===========================================================================
# I2C Base Class (an rewriten Adafruit_I2C pythone class clone)
# ===========================================================================

class I2C:
    def __init__(self, bus_no = 0):
        self.bus = smbus.SMBus(bus_no)

    def reverseByteOrder(self, data):
        # Reverses the byte order of an int (16-bit) or long (32-bit) value
        # Courtesy Vishal Sapre
        dstr = hex(data)[2:].replace('L','')
        byteCount = len(dstr[::2])
        val = 0
        for i, n in enumerate(range(byteCount)):
            d = data & 0xFF
            val |= (d << (8 * (byteCount - i - 1)))
            data >>= 8
        return val
    
    def readBit(self, addr, reg, bitNum):
        b = self.readU8(addr, reg)
        data = b & (1 << bitNum)
        return data
    
    def writeBit(self, addr, reg, bitNum, data):
        b = self.readU8(addr, reg)
        
        if data != 0:
            b = (b | (1 << bitNum))
        else:
            b = (b & ~(1 << bitNum))
            
        return self.write8(addr, reg, b)
    
    def readBits(self, addr, reg, bitStart, length):
        # 01101001 read byte
        # 76543210 bit numbers
        #    xxx   args: bitStart=4, length=3
        #    010   masked
        #   -> 010 shifted  
        
        b = self.readU8(addr, reg)
        mask = ((1 << length) - 1) << (bitStart - length + 1)
        b &= mask
        b >>= (bitStart - length + 1)
        
        return b
        
    def writeBits(self, addr, reg, bitStart, length, data):
        #      010 value to write
        # 76543210 bit numbers
        #    xxx   args: bitStart=4, length=3
        # 00011100 mask byte
        # 10101111 original value (sample)
        # 10100011 original & ~mask
        # 10101011 masked | value
        
        b = self.readU8(addr, reg)
        mask = ((1 << length) - 1) << (bitStart - length + 1)
        data <<= (bitStart - length + 1)
        data &= mask
        b &= ~(mask)
        b |= data
            
        return self.write8(addr, reg, b)

    def readBytes(self, addr, reg, length):
        output = []
        
        i = 0
        while i < length:
            output.append(self.readU8(addr, reg))
            i += 1
            
        return output        
        
    def readBytesListU(self, addr, reg, length):
        output = []
        
        i = 0
        while i < length:
            output.append(self.readU8(addr, reg + i))
            i += 1
            
        return output

    def readBytesListS(self, addr, reg, length):
        output = []
        
        i = 0
        while i < length:
            output.append(self.readS8(addr, reg + i))
            i += 1
            
        return output        
    
    def writeList(self, addr, reg, list):
        # Writes an array of bytes using I2C format"
        self.bus.write_i2c_block_data(addr, reg, list)
        
    def write8(self, addr, reg, value):
        # Writes an 8-bit value to the specified register/address
        self.bus.write_byte_data(addr, reg, value)
        
    def readU8(self, addr, reg):
            # Read an unsigned byte from the I2C device
            result = self.bus.read_byte_data(addr, reg)
            return result
        
    def readS8(self, addr, reg):
            # Reads a signed byte from the I2C device
            result = self.bus.read_byte_data(addr, reg)
            if result > 127:
                return result - 256
            else:
                return result
        
    def readU16(self, addr, reg):
            # Reads an unsigned 16-bit value from the I2C device
            hibyte = self.bus.read_byte_data(addr, reg)
            result = (hibyte << 8) + self.bus.read_byte_data(addr, reg + 1)
            return result
        
    def readS16(self, addr, reg):
            # Reads a signed 16-bit value from the I2C device
            hibyte = self.bus.read_byte_data(addr, reg)
            if hibyte > 127:
                hibyte -= 256
            result = (hibyte << 8) + self.bus.read_byte_data(addr, reg + 1)
            return result
        
    def read_byte(self, addr):
        return self.bus.read_byte(addr)
     
    def write_byte(self, addr, data):
        return self.bus.write_byte(addr, data)
    
    def read_reg_byte(self, addr, reg ):
    	return self.bus.read_byte_data(addr, reg) 
    
    def write_reg_byte(self, addr, reg, val):
	return self.bus.write_byte_data(addr, reg, val)
    
    def read_reg_block(self, addr, reg):
   	return self.bus.read_i2c_block_data(addr, reg)
    
    def write_reg_block(self, addr, req, data):
	return self.bus.write_i2c_block_data(self, addr, req, data)

    def read_block(self, addr, reg, len):
            return self.bus.read_i2c_block_data(addr, reg, len)
     
    def write_block(self, addr, reg, data):
            return self.bus.write_i2c_block_data(addr, reg, data)
                   
