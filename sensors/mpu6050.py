import math

def twos_compliment(high_byte, low_byte):
    value = (high_byte << 8) + low_byte
    if (value >= 0x8000):
        return -((0xffff - value) + 1)
    else:
        return value
        
class MPU6050(object):
    '''
    Simple MPU-6050 implementation
    '''

    PWR_MGMT_1 = 0x6b

    FS_SEL = 0x1b
    FS_250 = 0
    FS_500 = 1
    FS_1000 = 2
    FS_2000 = 3

    AFS_SEL = 0x1c
    AFS_2g = 0
    AFS_4g = 1
    AFS_8g = 2
    AFS_16g = 3

    ACCEL_START_BLOCK = 0x3b
    ACCEL_XOUT_H = 0
    ACCEL_XOUT_L = 1
    ACCEL_YOUT_H = 2
    ACCEL_YOUT_L = 3
    ACCEL_ZOUT_H = 4
    ACCEL_ZOUT_L = 5

    ACCEL_SCALE = { AFS_2g  : [ 2, 16384.0], AFS_4g  : [ 4, 8192.0], AFS_8g  : [ 8, 4096.0], AFS_16g : [16, 2048.0] }

    TEMP_START_BLOCK = 0x41
    TEMP_OUT_H = 0
    TEMP_OUT_L = 1

    GYRO_START_BLOCK = 0x43
    GYRO_XOUT_H = 0
    GYRO_XOUT_L = 1
    GYRO_YOUT_H = 2
    GYRO_YOUT_L = 3
    GYRO_ZOUT_H = 4
    GYRO_ZOUT_L = 5

    GYRO_SCALE = { FS_250  : [ 250, 131.0], FS_500  : [ 500, 65.5], FS_1000 : [1000, 32.8], FS_2000 : [2000, 16.4] }

    K = 0.98
    K1 = 1 - K

    def __init__(self, i2c, address, fs_scale=FS_250, afs_scale=AFS_2g):
        
        self.i2c = i2c
        self.address = address
        self.fs_scale = fs_scale
        self.afs_scale = afs_scale
        
        self.raw_gyro_data = [0, 0, 0, 0, 0, 0]
        self.raw_accel_data = [0, 0, 0, 0, 0, 0]
        self.raw_temp_data = [0, 0]
        
        self.gyro_raw_x = 0
        self.gyro_raw_y = 0
        self.gyro_raw_z = 0
        
        self.gyro_scaled_x = 0
        self.gyro_scaled_y = 0
        self.gyro_scaled_z = 0
        
        self.raw_temp = 0
        self.scaled_temp = 0
        
        self.accel_raw_x = 0
        self.accel_raw_y = 0
        self.accel_raw_z = 0
        
        self.accel_scaled_x = 0
        self.accel_scaled_y = 0
        self.accel_scaled_z = 0
        
        self.pitch = 0.0
        self.roll = 0.0
        
        # We need to wake up the module as it start in sleep mode
        self.i2c.write_byte(self.i2c, self.address, MPU6050.PWR_MGMT_1, 0)
        # Set the gryo resolution
        self.i2c.write_byte(self.i2c, self.address, MPU6050.FS_SEL, self.fs_scale << 3)
        # Set the accelerometer resolution
        self.i2c.write_byte(self.i2c, self.address, MPU6050.AFS_SEL, self.afs_scale << 3)
           
    def read_raw_data(self):
        '''
        Read the raw data from the sensor, scale it appropriately and store for later use
        '''
        self.raw_gyro_data = self.i2c.read_block(self.i2c, self.address, MPU6050.GYRO_START_BLOCK, 6)
        self.raw_accel_data = self.i2c.read_block(self.i2c, self.address, MPU6050.ACCEL_START_BLOCK, 6)
        self.raw_temp_data = self.i2c.read_block(self.i2c, self.address, MPU6050.TEMP_START_BLOCK, 2)
        
        self.gyro_raw_x = twos_compliment(self.raw_gyro_data[MPU6050.GYRO_XOUT_H], self.raw_gyro_data[MPU6050.GYRO_XOUT_L])
        self.gyro_raw_y = twos_compliment(self.raw_gyro_data[MPU6050.GYRO_YOUT_H], self.raw_gyro_data[MPU6050.GYRO_YOUT_L])
        self.gyro_raw_z = twos_compliment(self.raw_gyro_data[MPU6050.GYRO_ZOUT_H], self.raw_gyro_data[MPU6050.GYRO_ZOUT_L])
        
        self.accel_raw_x = twos_compliment(self.raw_accel_data[MPU6050.ACCEL_XOUT_H], self.raw_accel_data[MPU6050.ACCEL_XOUT_L])
        self.accel_raw_y = twos_compliment(self.raw_accel_data[MPU6050.ACCEL_YOUT_H], self.raw_accel_data[MPU6050.ACCEL_YOUT_L])
        self.accel_raw_z = twos_compliment(self.raw_accel_data[MPU6050.ACCEL_ZOUT_H], self.raw_accel_data[MPU6050.ACCEL_ZOUT_L])
        
        self.raw_temp = twos_compliment(self.raw_temp_data[MPU6050.TEMP_OUT_H], self.raw_temp_data[MPU6050.TEMP_OUT_L])

        # We convert these to radians for consistency and so we can easily combine later in the filter
        self.gyro_scaled_x = math.radians(self.gyro_raw_x / MPU6050.GYRO_SCALE[self.fs_scale][1]) 
        self.gyro_scaled_y = math.radians(self.gyro_raw_y / MPU6050.GYRO_SCALE[self.fs_scale][1]) 
        self.gyro_scaled_z = math.radians(self.gyro_raw_z / MPU6050.GYRO_SCALE[self.fs_scale][1]) 

        self.scaled_temp = self.raw_temp / 340 + 36.53

        self.accel_scaled_x = self.accel_raw_x / MPU6050.ACCEL_SCALE[self.afs_scale][1]
        self.accel_scaled_y = self.accel_raw_y / MPU6050.ACCEL_SCALE[self.afs_scale][1]
        self.accel_scaled_z = self.accel_raw_z / MPU6050.ACCEL_SCALE[self.afs_scale][1]
        
        self.pitch = self.read_x_rotation(self.read_scaled_accel_x(),self.read_scaled_accel_y(),self.read_scaled_accel_z())
        self.roll =  self.read_y_rotation(self.read_scaled_accel_x(),self.read_scaled_accel_y(),self.read_scaled_accel_z())
        
    def read_x_rotation(self, x, y, z):
        '''Returns the rotation around the X axis in radians'''
        return math.atan2(y, math.sqrt((x * x) + (z * z)))
    
    def read_y_rotation(self, x, y, z):
        '''Returns the rotation around the Y axis in radians'''
        return -math.atan2(x, math.sqrt((y * y) + (z * z)))
        
    def read_all(self):
        '''Return pitch and roll in radians and the scaled x, y & z values from the gyroscope and accelerometer'''
        self.read_raw_data()
        return (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)

if __name__=='__main__':

    import sys
    sys.path.append('..')

    from bus import I2C

    i2c = I2C(1)
    sensor = MPU6050(i2c)
    #sensor.initialize()
    
    while(True):
        print sensor.read_all()
        #print "Temperature(C): %.6f" % (baro.temp), "Pressure(millibar): %.6f" % (baro.press)
        time.sleep(0.1)
