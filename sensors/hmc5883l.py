
import math

from utils import *

class HMC5883L(object):
    '''
    Simple HMC5883L implementation
    '''
    TWO_PI = 2 * math.pi

    CONF_REG_A = 0
    CONF_REG_B = 1
    MODE_REG = 2
    DATA_START_BLOCK = 3
    DATA_XOUT_H = 0
    DATA_XOUT_L = 1
    DATA_ZOUT_H = 2
    DATA_ZOUT_L = 3
    DATA_YOUT_H = 4
    DATA_YOUT_L = 5

    SAMPLE_RATE = { 0 : 0.75, 1 : 1.5, 2 : 3, 3 : 7.5, 4 : 15, 5 : 30, 6 : 75, 7 :-1 }

    SAMPLE_MODE = { 0 : "CONTINUOUS", 1 : "SINGLE", 2 : "IDLE", 3 : "IDLE" }

    GAIN_SCALE = {
                    0 : [ 0.88, 1370, 0.73 ],
                    1 : [ 1.30, 1090, 0.92 ],
                    2 : [ 1.90, 820, 1.22 ],
                    3 : [ 2.50, 660, 1.52 ],
                    4 : [ 4.00, 440, 2.27 ],
                    5 : [ 4.70, 390, 2.56 ],
                    6 : [ 5.60, 330, 3.03 ],
                    7 : [ 8.10, 230, 4.35 ]
                 }


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
        self.i2c.write_reg_byte(self.address, HMC5883L.CONF_REG_A, conf_a)

        # Set the gain
        conf_b = self.gain << 5
        self.i2c.write_reg_byte(self.address, HMC5883L.CONF_REG_B, conf_b)

        # Set the operation mode
        self.i2c.write_reg_byte(self.address, HMC5883L.MODE_REG, self.sampling_mode)

        # Now read all the values as the first read after a gain change returns the old value
        self.read_raw_data()

    def update(self):
	self.read_raw_data()
	return (self.scaled_x, self.scaled_y, self.scaled_z) 

    def read_raw_data(self):
        '''
        Read the raw data from the sensor, scale it appropriately and store for later use
        '''
        self.raw_data = self.i2c.read_reg_block(self.address, HMC5883L.DATA_START_BLOCK)
        self.raw_x = twos_compliment(self.raw_data[HMC5883L.DATA_XOUT_H], self.raw_data[HMC5883L.DATA_XOUT_L]) - self.x_offset
        self.raw_y = twos_compliment(self.raw_data[HMC5883L.DATA_YOUT_H], self.raw_data[HMC5883L.DATA_YOUT_L]) - self.y_offset
        self.raw_z = twos_compliment(self.raw_data[HMC5883L.DATA_ZOUT_H], self.raw_data[HMC5883L.DATA_ZOUT_L]) - self.z_offset
    
        self.scaled_x = self.raw_x * HMC5883L.GAIN_SCALE[self.gain][2]
        self.scaled_y = self.raw_y * HMC5883L.GAIN_SCALE[self.gain][2]
        self.scaled_z = self.raw_z * HMC5883L.GAIN_SCALE[self.gain][2]

    def bearing(self):
        '''
        Read a bearing from the sensor assuming the sensor is level
        '''
        bearing = math.atan2(self.scaled_y, self.scaled_x)
        if bearing < 0:
            return bearing + (HMC5883L.TWO_PI)
        else:
            return bearing

    def compensated_bearing(self, pitch, roll):
        '''
        Calculate a bearing taking in to account the current pitch and roll of the device as supplied as parameters
        '''
        cos_pitch = (math.cos(pitch))
        sin_pitch = (math.sin(pitch))
        
        cos_roll = (math.cos(roll))
        sin_roll = (math.sin(roll))
    
        Xh = (self.scaled_x * cos_roll) + (self.scaled_z * sin_roll)
        Yh = (self.scaled_x * sin_pitch * sin_roll) + (self.scaled_y * cos_pitch) - (self.scaled_z * sin_pitch * cos_roll)
        
        bearing = math.atan2(Yh, Xh)
        if bearing < 0:
            return bearing + (HMC5883L.TWO_PI)
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
        print sensor.scaled_x, sensor.scaled_y, sensor.scaled_z 
	print sensor.bearing()* 180 / math.pi
	time.sleep(0.5)
	    
