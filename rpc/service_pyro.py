import sys
import Pyro4

sys.path.append("..")

from bus import *
from sensors import *

class IMUSensors(object):
    def init(self):
	self.i2c = I2C(1)
	self.mpu6050 = MPU6050(self.i2c)
        self.hmc5883 = HMC5883L(self.i2c)
	self.mpu6050.init()
	self.hmc5883.init()

    def update(self):
	self.mpu6050.update()
	self.hmc5883.update()
	return ((self.mpu6050.accel_scaled_x, self.mpu6050.accel_scaled_y,self.mpu6050.accel_scaled_z),
                (self.mpu6050.gyro_scaled_x, self.mpu6050.gyro_scaled_y, self.mpu6050.gyro_scaled_z),
                (self.hmc5883.scaled_x, self.hmc5883.scaled_y, self.hmc5883.scaled_z) ) 

nameserver = Pyro4.locateNS()
daemon = Pyro4.Daemon()                
uri = daemon.register(IMUSensors)   
nameserver.register("IMUSensors", uri)

print("Ready. Object uri =", uri)      
daemon.requestLoop()                
