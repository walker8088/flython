import time

class IMU(object):
    
    K = 0.98
    K1 = 1 - K
    
    def __init__(self, gyro, accel, compass):

        self.accel = accel
        self.gyro = gyro
        self.compass = compass)

        self.last_time = time.time()
        self.time_diff = 0

        self.pitch = 0
        self.roll = 0
        # take a reading from the device to allow it to settle after config changes
        self.read_all()
        # now take another to act a starting value
        self.read_all()
        self.pitch = self.rotation_x
        self.roll = self.rotation_y

    def read_all(self):
        '''Return pitch and roll in radians and the scaled x, y & z values from the gyroscope and accelerometer'''
        self.gyro.read_raw_data()
        self.accel.read_raw_data()
        
        self.gyro_scaled_x = self.gyro.gyro_scaled_x
        self.gyro_scaled_y = self.gyro.gyro_scaled_y
        self.gyro_scaled_z = self.gyro.gyro_scaled_z
        
        self.accel_scaled_x = self.accel.accel_scaled_x
        self.accel_scaled_y = self.accel.accel_scaled_y
        self.accel_scaled_z = self.accel.accel_scaled_z
        
        self.rotation_x = self.accel_scaled_x
        self.rotation_y = self.accel_scaled_y
        
        now = time.time()
        self.time_diff = now - self.last_time
        self.last_time = now 
        (self.pitch, self.roll) = self.comp_filter(self.rotation_x, self.rotation_y)
        
        # return (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        return (self.pitch, self.roll, self.gyro_scaled_x, self.gyro_scaled_y, self.gyro_scaled_z, self.accel_scaled_x, self.accel_scaled_y, self.accel_scaled_z)
        
    def read_x_rotation(self, x, y, z):   
        return self.rotation_x

    def read_y_rotation(self, x, y, z):
        return self.rotation_y

    def comp_filter(self, current_x, current_y):
        new_pitch = IMU.K * (self.pitch + self.gyro_scaled_x * self.time_diff) + (IMU.K1 * current_x)
        new_roll = IMU.K * (self.roll + self.gyro_scaled_y * self.time_diff) + (IMU.K1 * current_y)
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
