import math, time
from algorithm import *
 
class IMU(object):
    
    def __init__(self, gyro_accel, compass):

        self.gyro_accel = gyro_accel
        self.compass = compass
        
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        self.quad_fusion = QuadFusion() 
        self.dcm_fusion = DCMFusion()
    
    def init(self) :
        
        self.gyro_accel.init()
        self.compass.init()
        
        self.last_time = time.time()
        self.time_dt = 0
        
        self.gyro_accel.read()
        self.compass.read()
                             
    def read_all(self) :

        now = time.time()
        self.time_dt = now - self.last_time
        self.last_time = now 
	    
        self.gyro_accel.read()
        self.compass.read()

        self.accel_xyz = self.gyro_accel.accel_xyz()
        self.gyro_xyz = self.gyro_accel.gyro_xyz()
        self.compass_xyz = self.compass.compass_xyz()
	
    def update_quad(self):
        
        self.quad_fusion.update(self.accel_xyz, self.gyro_xyz, self.compass_xyz, self.time_dt)
        return (self.quad_fusion.pitch, self.quad_fusion.roll, self.quad_fusion.heading)

    def update_dcm(self):
        
        return self.dcm_fusion.update(self.accel_xyz, self.gyro_xyz, self.compass_xyz, self.time_dt)

    def set_compass_offsets(self,x_offset, y_offset, z_offset):
        
        self.compass.set_offsets(x_offset, y_offset, z_offset)

if __name__=='__main__':

    from bus import I2C
    from sensors import MPU6050, HMC5883L, MS5611
    
    i2c = I2C(1)
    
    gyro_accel = MPU6050(i2c)
    compass = HMC5883L(i2c)
    baro = MS5611(i2c)

    imu = IMU(gyro_accel, compass)
    imu.init()

    start_time = time.time()
    count = 0	
    while True :
        #vals = imu.update_quad()
	imu.read_all()
	vals = imu.update_dcm()
	vals = imu.update_quad()
	print "%.0f, %.0f, %.0f" % (math.degrees(vals[0]), math.degrees(vals[1]), math.degrees(vals[2]))
        count += 1
        #print imu.pitch, imu.roll
	#time.sleep(0.2)
    stop_time = time.time()
    print count / (stop_time-start_time)	
