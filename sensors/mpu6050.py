
import math

from utils import *
 
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

MPU6050_ACCEL_SCALE = { AFS_2g  : [ 2, 16384.0], AFS_4g  : [ 4, 8192.0], AFS_8g  : [ 8, 4096.0], AFS_16g : [16, 2048.0] }

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

MPU6050_GYRO_SCALE = { FS_250  : [ 250, 131.0], FS_500  : [ 500, 65.5], FS_1000 : [1000, 32.8], FS_2000 : [2000, 16.4] }

class MPU6050(object):
    '''
    Simple MPU-6050 implementation
    '''

    
    K = 0.98
    K1 = 1 - K

    def __init__(self, i2c, address=0x68 ):
        
        self.i2c = i2c
        self.address = address
        
        self.fs_scale = FS_250
        self.afs_scale = AFS_2g
        
        self.raw_gyro_data = [0, 0, 0, 0, 0, 0]
        self.raw_accel_data = [0, 0, 0, 0, 0, 0]
        self.raw_temp_data = [0, 0]
        
        self.gyro_raw_x = 0
        self.gyro_raw_y = 0
        self.gyro_raw_z = 0
        
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        
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
        self.raw_gyro_data = self.i2c.read_reg_block(self.address, MPU6050_GYRO_START_BLOCK)
        self.raw_accel_data = self.i2c.read_reg_block(self.address, MPU6050_ACCEL_START_BLOCK)
        self.raw_temp_data = self.i2c.read_reg_block(self.address, MPU6050_TEMP_START_BLOCK)
        
        self.gyro_raw_x = twos_compliment(self.raw_gyro_data[MPU6050_GYRO_XOUT_H], self.raw_gyro_data[MPU6050_GYRO_XOUT_L])
        self.gyro_raw_y = twos_compliment(self.raw_gyro_data[MPU6050_GYRO_YOUT_H], self.raw_gyro_data[MPU6050_GYRO_YOUT_L])
        self.gyro_raw_z = twos_compliment(self.raw_gyro_data[MPU6050_GYRO_ZOUT_H], self.raw_gyro_data[MPU6050_GYRO_ZOUT_L])
        
        self.accel_raw_x = twos_compliment(self.raw_accel_data[MPU6050_ACCEL_XOUT_H], self.raw_accel_data[MPU6050_ACCEL_XOUT_L])
        self.accel_raw_y = twos_compliment(self.raw_accel_data[MPU6050_ACCEL_YOUT_H], self.raw_accel_data[MPU6050_ACCEL_YOUT_L])
        self.accel_raw_z = twos_compliment(self.raw_accel_data[MPU6050_ACCEL_ZOUT_H], self.raw_accel_data[MPU6050_ACCEL_ZOUT_L])
        
        self.raw_temp = twos_compliment(self.raw_temp_data[MPU6050_TEMP_OUT_H], self.raw_temp_data[MPU6050_TEMP_OUT_L])

        # We convert these to radians for consistency and so we can easily combine later in the filter
        self.gyro_x = math.radians(self.gyro_raw_x / MPU6050_GYRO_SCALE[self.fs_scale][1]) 
        self.gyro_y = math.radians(self.gyro_raw_y / MPU6050_GYRO_SCALE[self.fs_scale][1]) 
        self.gyro_z = math.radians(self.gyro_raw_z / MPU6050_GYRO_SCALE[self.fs_scale][1]) 

        self.accel_x = self.accel_raw_x / MPU6050_ACCEL_SCALE[self.afs_scale][1]
        self.accel_y = self.accel_raw_y / MPU6050_ACCEL_SCALE[self.afs_scale][1]
        self.accel_z = self.accel_raw_z / MPU6050_ACCEL_SCALE[self.afs_scale][1]

        self.temp = self.raw_temp / 340 + 36.53
                
    def update(self):
        self.read()
        return (self.gyro_x, self.gyro_y, self.gyro_z, self.accel_x, self.accel_y, self.accel_z)

if __name__=='__main__':

    import sys,time

    sys.path.append('..')

    from bus import i2c

    i2c = I2C(1)
    sensor = MPU6050(i2c)
    #sensor.initialize()
    
    while(True):
        print sensor.update()
        time.sleep(0.1)
