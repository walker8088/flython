

import math 

# origin from http://www.pieter-jan.com/node/11

class DCMFusion(object):
    
    def __init__(self):
        self.pitch = 0.0
        self.roll = 0.0		
        self.yaw = 0.0
        self.K = 0.95

    def update_imu(self, accel_xyz, gyro_xyz, compass_xyz, time_dt):

    	self.pitch += -gyro_xyz[1] * time_dt 
    	self.roll += gyro_xyz[0] * time_dt
        self.yaw += -gyro_xyz[2] * time_dt

    	#Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    	#force = math.fabs(accel_xyz[0]) + math.fabs(accel_xyz[1])  + math.fabs(accel_xyz[2])
    	#print force
    	#if (force >= 0.5) and (force <= 2.5):
        
    	roll_acc = math.atan2(accel_xyz[1], accel_xyz[2])
    	pitch_acc = math.atan2(accel_xyz[0], accel_xyz[2])

    	self.pitch = self.pitch * self.K + pitch_acc * (1 - self.K)
    	self.roll = self.roll * self.K  + roll_acc * (1 - self.K)
        compass_yaw = self.compensated_yaw(self.pitch, self.roll, compass_xyz)  
        #self.yaw = self.yaw * self.K + compass_yaw * (1 - self.K)

        '''
        if self.roll < 0.0 : 
            self.roll += math.pi * 2
        if self.pitch < 0.0 :
            self.pitch += math.pi * 2
        if self.yaw < 0.0 :
            self.yaw += math.pi * 2
        '''

    	return (self.pitch, self.roll, self.yaw) 
 
    def compensated_yaw(self, pitch, roll, compass_xyz):
        '''
        Calculate a bearing taking in to account the current pitch and roll of the device as supplied as parameters
        '''
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
    
        Xh = (compass_xyz[0] * cos_roll) + (compass_xyz[2] * sin_roll)
        Yh = (compass_xyz[0] * sin_pitch * sin_roll) + (compass_xyz[1] * cos_pitch) - (compass_xyz[2] * sin_pitch * cos_roll)
        
        yaw = math.atan2(Yh, Xh)

        if yaw < 0:
            yaw += math.pi * 2
        
        return yaw

