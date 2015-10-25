import sys
import rpyc

sys.path.append("..")

from bus import *
from sensors import *

class IMUSensorsService(rpyc.Service):
    
    def on_connect(self):
        # code that runs when a connection is created
        # (to init the serivce, if needed)
        pass

    def on_disconnect(self):
        # code that runs when the connection has already closed
        # (to finalize the service, if needed)
        pass

    def exposed_init(self):
	self.i2c = I2C(1)
	self.mpu6050 = MPU6050(self.i2c)
        self.hmc5883 = HMC5883L(self.i2c)
	self.mpu6050.init()
	self.hmc5883.init()

    def exposed_update(self):
        self.mpu6050.update()
	self.hmc5883.update()
	return (self.mpu6050.accel_scaled_x, self.mpu6050.accel_scaled_y,self.mpu6050.accel_scaled_z,
                self.mpu6050.gyro_scaled_x, self.mpu6050.gyro_scaled_y, self.mpu6050.gyro_scaled_z,
                self.hmc5883.scaled_x, self.hmc5883.scaled_y, self.hmc5883.scaled_z ) 

if __name__ == "__main__":
    from rpyc.utils.server import ThreadedServer
    t = ThreadedServer(IMUSensorsService, port = 5678)
    t.start()
