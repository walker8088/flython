
import sys
import math

class AHRS(object):
        def __init__(self, imu_fusion) :
		
		self.pitch = 0.0
		self.roll  = 0.0
		self.yaw   = 0.0
		self.altitude = 0.0

		self.declination = 0.0
		self.accel_xyz = (0.0, 0.0, 0.0)
		self.gyro_xyz  = (0.0, 0.0, 0.0)
		self.compass_xyz = (0.0, 0.0, 0.0)
		self.baro_press = 0.0
		self.baro_temp = 0.0
		
		self.altitude_base = 0.0

		self.imu_fusion = imu_fusion

        def update_imu(accel_xyz, gyro_xyz, compass_xyz, time_dt):	

		self.accel_xyz = accel_xyz
		self.gyro_xyz = gyro_xyz
		self.compass_xyz = compass_xyz

		self.pitch, self.roll, self.yaw = self.imu_fusion.update(accel_xyz, gyro_xyz, compass_xyz, time_dt)
		
		return (self.pitch, self.roll, self.yaw)

	def update_baro(self, baro_press, baro_temp):
		
		self.baro_press = baro_press
		self.baro_temp = baro_temp
		
		new_altitude = ((math.pow((101325.0 / self.baro_press), 1/5.257) - 1.0) * (self.baro_temp + 273.15)) / 0.0065
		self.altitude = new_altitude - self.altitude_base

		return self.altitude

	def set_altitude_base(self) :
		self.altitude_base = self.altitude

	def update_gps(self, gps):
		pass
		