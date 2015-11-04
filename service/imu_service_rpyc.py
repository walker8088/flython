#!/usr/bin/python

import sys, time
import rpyc

sys.path.append("..")

from bus import *
from sensors import *
from algorithm import *

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

    def exposed_init_quad(self):
        
        self.exposed_init()
        self.quat_fusion = QuaternionFusion()

    def exposed_update(self):
        
        new_time = time.time()
        time_dt = new_time - self.last_time
        self.last_time = new_time
        
        self.mpu6050.update()
        self.hmc5883.update()

        return (self.mpu6050.accel_xyz(), self.mpu6050.gyro_xyz(), self.hmc5883.compass_xyz(), time_dt ) 
    
    def exposed_update_quat(self):
        
        accel_xyz, gyro_xyz, compass_xyz, time_dt = self.exposed_update()
        self.quat_fusion.update_imu(accel_xyz, gyro_xyz, compass_xyz, time_dt)

        return self.quad_fusion.q

if __name__ == "__main__":
    from rpyc.utils.server import ThreadedServer
    t = ThreadedServer(IMUSensorsService, port = 5678)
    t.start()
