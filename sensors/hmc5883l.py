
from math import sin, cos, atan2, pi, degrees 

from utils import *

HMC5883L_CONF_REG_A = 0
HMC5883L_CONF_REG_B = 1
HMC5883L_MODE_REG = 2
HMC5883L_DATA_START_BLOCK = 3
HMC5883L_DATA_XOUT_H = 0
HMC5883L_DATA_XOUT_L = 1
HMC5883L_DATA_ZOUT_H = 2
HMC5883L_DATA_ZOUT_L = 3
HMC5883L_DATA_YOUT_H = 4
HMC5883L_DATA_YOUT_L = 5

HMC5883L_SAMPLE_RATE = { 0 : 0.75, 1 : 1.5, 2 : 3, 3 : 7.5, 4 : 15, 5 : 30, 6 : 75, 7 :-1 }
HMC5883L_SAMPLE_MODE = { 0 : "CONTINUOUS", 1 : "SINGLE", 2 : "IDLE", 3 : "IDLE" }

HMC5883L_GAIN_SCALE = {
                0 : [ 0.88, 1370, 0.73 ],
                1 : [ 1.30, 1090, 0.92 ],
                2 : [ 1.90, 820, 1.22 ],
                3 : [ 2.50, 660, 1.52 ],
                4 : [ 4.00, 440, 2.27 ],
                5 : [ 4.70, 390, 2.56 ],
                6 : [ 5.60, 330, 3.03 ],
                7 : [ 8.10, 230, 4.35 ]
             }

class HMC5883L(object):

    def __init__(self, i2c, samples=3, rate=4, gain=1, sampling_mode=0):
        self.i2c = i2c
        self.address = 0x1e
        self.samples = samples
        self.rate = rate
        self.gain = gain
        self.sampling_mode = sampling_mode
        
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
            
    def init(self):
        #ugly hack here, cause hmmc5883l slave to mpu6050
        self.i2c.write_reg_byte(0x68, 0x6A, 0);
        self.i2c.write_reg_byte(0x68, 0x37, 2);
        self.i2c.write_reg_byte(0x68, 0x6B, 0);
 
         # Set the number of samples
        conf_a = (self.samples << 5) + (self.rate << 2)
        self.i2c.write_reg_byte(self.address, HMC5883L_CONF_REG_A, conf_a)

        # Set the gain
        conf_b = self.gain << 5
        self.i2c.write_reg_byte(self.address, HMC5883L_CONF_REG_B, conf_b)

        # Set the operation mode
        self.i2c.write_reg_byte(self.address, HMC5883L_MODE_REG, self.sampling_mode)

        # Now read all the values as the first read after a gain change returns the old value
        self.read()

    def compass_xyz(self):
        return (self.compass_x,self.compass_y,self.compass_z) 

    def update(self):
	   self.read()
	   return (self.compass_x, self.compass_y, self.compass_z) 

    def read(self):
        '''
        Read the raw data from the sensor, scale it appropriately and store for later use
        '''
        raw_data = self.i2c.read_reg_block(self.address, HMC5883L_DATA_START_BLOCK)

        self.raw_x = bytes_to_int(raw_data[HMC5883L_DATA_XOUT_H], raw_data[HMC5883L_DATA_XOUT_L]) - self.x_offset
        self.raw_y = bytes_to_int(raw_data[HMC5883L_DATA_YOUT_H], raw_data[HMC5883L_DATA_YOUT_L]) - self.y_offset
        self.raw_z = bytes_to_int(raw_data[HMC5883L_DATA_ZOUT_H], raw_data[HMC5883L_DATA_ZOUT_L]) - self.z_offset
        
        gain_value = HMC5883L_GAIN_SCALE[self.gain][2] 
        self.compass_x = self.raw_x * gain_value
        self.compass_y = self.raw_y * gain_value
        self.compass_z = self.raw_z * gain_value

    def bearing(self):
        '''
        Read a bearing from the sensor assuming the sensor is level
        '''
        bearing = atan2(self.compass_y, self.compass_x)
        if bearing < 0:
            return bearing + 2 * pi
        else:
            return bearing

    def compensated_bearing(self, pitch, roll):
        '''
        Calculate a bearing taking in to account the current pitch and roll of the device as supplied as parameters
        '''
        cos_pitch = cos(pitch)
        sin_pitch = sin(pitch)
        
        cos_roll = cos(roll)
        sin_roll = sin(roll)
    
        Xh = (self.compass_x * cos_roll) + (self.compass_z * sin_roll)
        Yh = (self.compass_x * sin_pitch * sin_roll) + (self.compass_y * cos_pitch) - (self.compass_z * sin_pitch * cos_roll)
        
        bearing = atan2(Yh, Xh)
        if bearing < 0:
            return bearing + pi * 2 
        else:
            return bearing
    
    def set_offsets(self, x_offset, y_offset, z_offset):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
    
if __name__=='__main__':
    import sys,time
    sys.path.append('..')
    from bus import I2C
    i2c = I2C(1)
    sensor = HMC5883L(i2c)
    sensor.init()
    while True:
        sensor.update()
        #print sensor.compass_x, sensor.compass_y, sensor.compass_z 
	print degrees(sensor.bearing())
	time.sleep(0.5)
	    
