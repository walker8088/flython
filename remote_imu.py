<<<<<<< HEAD

import sys
import time
import threading
import rpyc

sys.path.append("../flython/")
from ahrs import *

class RemoteIMU():
    def __init__(self, host, port, observer):
        self.host = host
        self.port  = port
        self.observer = observer
        
        self.conn = None
        self.running = False
                        
    def open(self):
        
        if self.conn :
                return False
                
        self.conn = rpyc.connect(self.host, self.port)
        self.raw_imu = self.conn.root
        self.raw_imu.init()
        self.fusion = QuaternionFusion()
        #self.fusion = DCMFusion()
        
        # run main loop as a thread
        self.loop_thread = threading.Thread(target = self.loop_run)
        self.loop_thread.daemon = True
        self.running = True
        self.loop_thread.start()
        
        return True
        
    def close(self):
        if not self.conn:
            return
        self.running = False    
        while self.conn :
                time.sleep(0.2)
        
    def loop_run(self):
        while self.running : 
             accel_xyz, gyro_xyz, compass_xyz, time_dt = self.raw_imu.update()
             roll, pitch, yaw = self.fusion.update(accel_xyz, gyro_xyz, compass_xyz, time_dt)
             self.observer.update_attitude(pitch, roll, yaw)
             time.sleep(0.2)
        #end while
        self.conn.close()
        self.conn = None 
=======

import sys
import time
import threading
import rpyc

sys.path.append("../flython/")
from ahrs import *

class RemoteIMU():
    def __init__(self, host, port, observer):
        self.host = host
        self.port  = port
        self.observer = observer
        
        self.conn = None
        self.running = False
                        
    def open(self):
        
        if self.conn :
                return False
                
        self.conn = rpyc.connect(self.host, self.port)
        self.raw_imu = self.conn.root
        self.raw_imu.init()
        self.fusion = QuaternionFusion()
        #self.fusion = DCMFusion()
        
        # run main loop as a thread
        self.loop_thread = threading.Thread(target = self.loop_run)
        self.loop_thread.daemon = True
        self.running = True
        self.loop_thread.start()
        
        return True
        
    def close(self):
        if not self.conn:
            return
        self.running = False    
        while self.conn :
                time.sleep(0.2)
        
    def loop_run(self):
        while self.running : 
             accel_xyz, gyro_xyz, compass_xyz, time_dt = self.raw_imu.update()
             roll, pitch, yaw = self.fusion.update(accel_xyz, gyro_xyz, compass_xyz, time_dt)
             self.observer.update_attitude(pitch, roll, yaw)
             time.sleep(0.2)
        #end while
        self.conn.close()
        self.conn = None 
>>>>>>> 781bcee13012c2094b8a1dda3f292621b59e4557
        