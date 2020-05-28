
import pyb
from pyb import I2C

class I2C_MP:
    def __init__(self, bus_no = 1):
        self.bus = pyb.I2C(bus_no, I2C.MASTER)

    def read_byte(self, addr):
        return self.bus.recv(1, addr)
     
    def read_mem_byte(self, addr, mem_addr):
    	return self.bus.mem_read(1, addr, mem_addr) 
    
    def read_mem_block(self, addr, mem_addr, len):
	return self.bus.mem_read(len, addr, mem_addr)
   
    def write_byte(self, addr, data):
        return self.bus.send(data, addr)
    
    def write_mem_byte(self, addr, mem_addr, val):
	return self.bus.mem_write(val, addr, mem_addr)
    
    def write_mem_block(self, addr, mem_addr, data):
	return self.bus.mem_write(data, addr, mem_addr)