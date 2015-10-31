#!/usr/bin/python

import sys, time
import rpyc

sys.path.append("..")

#from bus import *
#from sensors import *

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
        self.last_time = time.time()

    def exposed_update(self):
        
        new_time = time.time()
        time_dt = new_time - self.last_time
        self.last_time = new_time
        
        self.mpu6050.update()
        self.hmc5883.update()

        return (self.mpu6050.accel_x, self.mpu6050.accel_y,self.mpu6050.accel_z,
                self.mpu6050.gyro_x, self.mpu6050.gyro_y, self.mpu6050.gyro_z,
                self.hmc5883.compass_x, self.hmc5883.compass_y, self.hmc5883.compass_z, time_dt ) 

if __name__ == "__main__":
    from rpyc.utils.server import ThreadedServer
    t = ThreadedServer(IMUSensorsService, port = 5678)
    t.start()
