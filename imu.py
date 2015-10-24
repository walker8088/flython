import time
from algorithm import *
 
class IMU(object):
    
    K = 0.98
    K1 = 1 - K
    
    def __init__(self, gyro_accel, compass, baro):

        self.gyro_accel = gyro_accel
        self.compass = compass
	self.baro = baro

        self.last_time = time.time()
        self.time_diff = 0

        self.pitch = 0
        self.roll = 0
        # take a reading from the device to allow it to settle after config changes
        self.read_all()
        # now take another to act a starting value
        self.read_all()
        #self.pitch = self.rotation_x
        #self.roll = self.rotation_y
        
	self.fusion = Fusion() 
    
    def init(self) :
	self.gyro_accel.init()
	self.compass.init()
	self.baro.init()
	
    def update(self): 
	self.read_all()
	self.fusion.update(self.accel_xyz, self.gyro_xyz, self.compass_xyz, 0.01)
	self.baro.update()

    def read_all(self):
        '''Return pitch and roll in radians and the scaled x, y & z values from the gyroscope and accelerometer'''
        self.gyro_accel.read_raw_data()
	self.compass.read_raw_data()
       
        self.gyro_xyz = (self.gyro_accel.gyro_scaled_x, self.gyro_accel.gyro_scaled_y, self.gyro_accel.gyro_scaled_z)
        self.accel_xyz = (self.gyro_accel.accel_scaled_x, self.gyro_accel.accel_scaled_y, self.gyro_accel.accel_scaled_z)
	self.compass_xyz = (self.compass.scaled_x, self.compass.scaled_y, self.compass.scaled_z)
        
        self.rotation_xy = (self.gyro_accel.accel_scaled_x, self.gyro_accel.accel_scaled_y)
        
        now = time.time()
        self.time_diff = now - self.last_time
        self.last_time = now 
        (self.pitch, self.roll) = self.comp_filter(self.rotation_xy)
        
        # return (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        #return (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        
    def read_x_rotation(self, x, y, z):   
        return self.rotation_x

    def read_y_rotation(self, x, y, z):
        return self.rotation_y

    def comp_filter(self, current_xy):
        current_x,current_y = current_xy
        new_pitch = IMU.K * (self.pitch + self.gyro_xyz[0] * self.time_diff) + (IMU.K1 * current_x)
        new_roll = IMU.K * (self.roll + self.gyro_xyz[1] * self.time_diff) + (IMU.K1 * current_y)
        return (new_pitch, new_roll)


    def read_pitch_roll_yaw(self):
        '''
        Return pitch, roll and yaw in radians
        '''
        (raw_pitch, raw_roll, self.gyro_scaled_x, self.gyro_scaled_y, \
            self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, \
            self.accel_scaled_z) = self.read_all()
        
        now = time.time()
        self.time_diff = now - self.last_time
        self.last_time = now 
        
        (self.pitch, self.roll) = self.comp_filter(raw_pitch, raw_roll)
        self.yaw = self.compass.read_compensated_bearing(self.pitch, self.roll)
        
        return (self.pitch, self.roll, self.yaw)

    def set_compass_offsets(self,x_offset, y_offset, z_offset):
        self.compass.set_offsets(x_offset, y_offset, z_offset)

if __name__=='__main__':

    from bus import I2C
    from sensors import MPU6050, HMC5883L, MS5611
    
    i2c = I2C(1)
    
    gyro_accel = MPU6050(i2c)
    compass = HMC5883L(i2c)
    baro = MS5611(i2c)

    imu = IMU(gyro_accel, compass, baro)
    imu.init()

    start_time = time.time()
    count = 0	
    while count < 10000 :
	imu.update()
        count += 1
        #print imu.pitch, imu.roll
	#time.sleep(0.2)
    stop_time = time.time()
    print count / (stop_time-start_time)	
