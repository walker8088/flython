
import math

from utils import bytes_to_int
 
MPU6050_PWR_MGMT_1 = 0x6b

MPU6050_FS_SEL = 0x1b
MPU6050_FS_250 = 0
MPU6050_FS_500 = 1
MPU6050_FS_1000 = 2
MPU6050_FS_2000 = 3

MPU6050_AFS_SEL = 0x1c
MPU6050_AFS_2g = 0
MPU6050_AFS_4g = 1
MPU6050_AFS_8g = 2
MPU6050_AFS_16g = 3

MPU6050_ACCEL_START_BLOCK = 0x3b
MPU6050_ACCEL_XOUT_H = 0
MPU6050_ACCEL_XOUT_L = 1
MPU6050_ACCEL_YOUT_H = 2
MPU6050_ACCEL_YOUT_L = 3
MPU6050_ACCEL_ZOUT_H = 4
MPU6050_ACCEL_ZOUT_L = 5

MPU6050_ACCEL_SCALE = { 
	MPU6050_AFS_2g  : [ 2, 16384.0], 
	MPU6050_AFS_4g  : [ 4, 8192.0], 
	MPU6050_AFS_8g  : [ 8, 4096.0], 
	MPU6050_AFS_16g : [16, 2048.0] 
	}

MPU6050_TEMP_START_BLOCK = 0x41
MPU6050_TEMP_OUT_H = 0
MPU6050_TEMP_OUT_L = 1

MPU6050_GYRO_START_BLOCK = 0x43
MPU6050_GYRO_XOUT_H = 0
MPU6050_GYRO_XOUT_L = 1
MPU6050_GYRO_YOUT_H = 2
MPU6050_GYRO_YOUT_L = 3
MPU6050_GYRO_ZOUT_H = 4
MPU6050_GYRO_ZOUT_L = 5

MPU6050_GYRO_SCALE = { 
	MPU6050_FS_250  : [250, 131.0], 
	MPU6050_FS_500  : [500, 65.5], 
	MPU6050_FS_1000 : [1000, 32.8], 
	MPU6050_FS_2000 : [2000, 16.4] 
	}

class MPU6050(object):

    def __init__(self, i2c, address=0x68 ):
        
        self.i2c = i2c
        self.address = address
        
        self.fs_scale = MPU6050_FS_250
        self.afs_scale = MPU6050_AFS_2g
        
        self.gyro_raw_x = 0
        self.gyro_raw_y = 0
        self.gyro_raw_z = 0
        
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        
        self.gyro_x_offset = 0
        self.gyro_y_offset = 0
        self.gyro_z_offset = 0

        self.raw_temp = 0
        self.temp = 0
        
        self.accel_raw_x = 0
        self.accel_raw_y = 0
        self.accel_raw_z = 0
        
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0

    def gyro_xyz(self):    
        return (self.gyro_x, self.gyro_y, self.gyro_z) 
    
    def accel_xyz(self):
        return (self.accel_x, self.accel_y, self.accel_z)
            
    def init(self):    
        # We need to wake up the module as it start in sleep mode
        self.i2c.write_reg_byte(self.address, MPU6050_PWR_MGMT_1, 0)
        # Set the gryo resolution
        self.i2c.write_reg_byte(self.address, MPU6050_FS_SEL, self.fs_scale << 3)
        # Set the accelerometer resolution
        self.i2c.write_reg_byte(self.address, MPU6050_AFS_SEL, self.afs_scale << 3)
        #TODO bypass mode
	   
    def read(self):
        '''
        Read the raw data from the sensor, scale it appropriately and store for later use
        '''
        raw_gyro_data = self.i2c.read_reg_block(self.address, MPU6050_GYRO_START_BLOCK)
        raw_accel_data = self.i2c.read_reg_block(self.address, MPU6050_ACCEL_START_BLOCK)
        raw_temp_data = self.i2c.read_reg_block(self.address, MPU6050_TEMP_START_BLOCK)
        
        self.gyro_raw_x = bytes_to_int(raw_gyro_data[MPU6050_GYRO_XOUT_H], raw_gyro_data[MPU6050_GYRO_XOUT_L])
        self.gyro_raw_y = bytes_to_int(raw_gyro_data[MPU6050_GYRO_YOUT_H], raw_gyro_data[MPU6050_GYRO_YOUT_L])
        self.gyro_raw_z = bytes_to_int(raw_gyro_data[MPU6050_GYRO_ZOUT_H], raw_gyro_data[MPU6050_GYRO_ZOUT_L])
        
        self.accel_raw_x = bytes_to_int(raw_accel_data[MPU6050_ACCEL_XOUT_H], raw_accel_data[MPU6050_ACCEL_XOUT_L])
        self.accel_raw_y = bytes_to_int(raw_accel_data[MPU6050_ACCEL_YOUT_H], raw_accel_data[MPU6050_ACCEL_YOUT_L])
        self.accel_raw_z = bytes_to_int(raw_accel_data[MPU6050_ACCEL_ZOUT_H], raw_accel_data[MPU6050_ACCEL_ZOUT_L])
        
        self.raw_temp = bytes_to_int(raw_temp_data[MPU6050_TEMP_OUT_H], raw_temp_data[MPU6050_TEMP_OUT_L])

        # We convert these to radians for consistency and so we can easily combine later in the filter
        self.gyro_x = math.radians(self.gyro_raw_x / MPU6050_GYRO_SCALE[self.fs_scale][1]) - self.gyro_x_offset
        self.gyro_y = math.radians(self.gyro_raw_y / MPU6050_GYRO_SCALE[self.fs_scale][1]) - self.gyro_y_offset
        self.gyro_z = math.radians(self.gyro_raw_z / MPU6050_GYRO_SCALE[self.fs_scale][1]) - self.gyro_z_offset

        self.accel_x = self.accel_raw_x / MPU6050_ACCEL_SCALE[self.afs_scale][1]
        self.accel_y = self.accel_raw_y / MPU6050_ACCEL_SCALE[self.afs_scale][1]
        self.accel_z = self.accel_raw_z / MPU6050_ACCEL_SCALE[self.afs_scale][1]

        self.temp = self.raw_temp / 340 + 36.53
    
    def auto_gyro_offset(self, count) :
        x_list = []
        y_list = []
        z_list = []

        for i in range(count) :
            self.read()
            x_list.append(self.gyro_x)
            y_list.append(self.gyro_y)		    	
            z_list.append(self.gyro_z)

        self.gyro_x_offset = sum(x_list) / count
        self.gyro_y_offset = sum(y_list) / count
        self.gyro_z_offset = sum(z_list) / count

        return (self.gyro_x_offset, self.gyro_y_offset, self.gyro_z_offset)

    def update(self):
        self.read()
        return (self.gyro_x, self.gyro_y, self.gyro_z, self.accel_x, self.accel_y, self.accel_z)

if __name__=='__main__':

    import sys,time

    sys.path.append('..')

    from bus import *

    i2c = I2C(1)
    sensor = MPU6050(i2c)
    sensor.init()
    
    print sensor.auto_gyro_offset(count = 100)

    while(True):
        print sensor.update()
        time.sleep(0.1)
	break
