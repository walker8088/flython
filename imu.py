import time
from algorithm import *
 
class IMU(object):
    
    K = 0.98
    K1 = 1 - K
    
    def __init__(self, gyro_accel, compass, baro):

        self.gyro_accel = gyro_accel
        self.compass = compass
        self.baro = baro

        self.pitch = 0
        self.roll = 0
        
        self.fusion = Fusion() 
    
    def init(self) :
        
        self.gyro_accel.init()
        self.compass.init()
        #self.baro.init()
	    
        self.last_time = time.time()
        self.time_dt = 0
        
        self.gyro_accel.read()
        self.compass.read()
        
    def update(self): 
        
        now = time.time()
        self.time_dt = now - self.last_time
        self.last_time = now 
        
        self.gyro_accel.read()
        self.compass.read()
        self.gyro_xyz = self.gyro_accel.gyro_xyz()
        
        
        #(self.pitch, self.roll) = self.comp_filter(self.rotation_xy)
        
        self.fusion.update(self.gyro_accel.accel_xyz(), self.gyro_accel.gyro_xyz(), self.compass.compass_xyz(), self.time_dt)
        
        return (self.fusion.pitch, self.fusion.roll, self.fusion.yaw)

    def comp_filter(self, current_xy):
        current_x,current_y = current_xy
        new_pitch = IMU.K * (self.pitch + self.gyro_xyz[0] * self.time_dt) + (IMU.K1 * current_x)
        new_roll = IMU.K * (self.roll + self.gyro_xyz[1] * self.time_dt) + (IMU.K1 * current_y)
        return (new_pitch, new_roll)


    def read_pitch_roll_yaw(self):
        '''
        Return pitch, roll and yaw in radians
        '''
        (raw_pitch, raw_roll, self.gyro_scaled_x, self.gyro_scaled_y, \
            self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, \
            self.accel_scaled_z) = self.read_all()
        
        now = time.time()
        self.time_dt = now - self.last_time
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
        print imu.update()
        count += 1
        #print imu.pitch, imu.roll
	#time.sleep(0.2)
    stop_time = time.time()
    print count / (stop_time-start_time)	
