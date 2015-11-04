
import math

'''
comfirm from https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_.E2.86.94_quaternion
'''

class QuaternionFusion(object):

    def __init__(self):
        self.q = (1, 0.0, 0.0, 0.0)
        self.twoKi = 0
        self.twoKp = 5.0
        
        self.integralFBx = 0.0
        self.integralFBy = 0.0
        self.integralFBz = 0.0

    def update_imu(self, accel_xyz, gyro_xyz, mag_xyz, time_dt):

        #self.update_no_mag(accel_xyz, gyro_xyz, time_dt)
        self.update(accel_xyz, gyro_xyz, mag_xyz, time_dt)
        return self.euler()

    def euler(self):

        q0, q1, q2, q3 = self.q

        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1-2*(q1*q1+q2*q2))
        pitch = -math.asin(2 * (q0 * q2-q3 * q1))
        yaw = -math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2*q2+q3*q3))
        
        if roll < 0.0 : 
            roll += math.pi * 2
        if pitch < 0.0 :
            pitch += math.pi * 2
        if yaw < 0.0 :
            yaw += math.pi * 2
        
        return (pitch, roll, yaw)

    def update(self, accel_xyz, gyro_xyz, mag_xyz, time_dt):

        q0, q1, q2, q3 = self.q  
        ax, ay, az = accel_xyz
        gx, gy, gz = gyro_xyz
        mx, my, mz = mag_xyz

        ## Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        #if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        #    updateIMU(gx, gy, gz, ax, ay, az, dt)
        #    return
        #}

        
        #Normalise accelerometer measurement
        normal = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax *= normal
        ay *= normal
        az *= normal

        #Normalise magnetometer measurement
        normal = 1.0 / math.sqrt(mx * mx + my * my + mz * mz)
        mx *= normal
        my *= normal
        mz *= normal

        # Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        # Reference direction of Earth's magnetic field
        hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
        hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
        bx = math.sqrt(hx * hx + hy * hy)
        bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))

        # Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2
        halfvy = q0q1 + q2q3
        halfvz = q0q0 - 0.5 + q3q3
        halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)

        # Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

        # Compute and apply integral feedback if enabled
        if self.twoKi > 0.0 :
            self.integralFBx += self.twoKi * halfex * time_dt # integral error scaled by Ki
            self.integralFBy += self.twoKi * halfey * time_dt
            self.integralFBz += self.twoKi * halfez * time_dt
            gx += self.integralFBx  # apply integral feedback
            gy += self.integralFBy
            gz += self.integralFBz
        else: 
            self.integralFBx = 0.0 # prevent integral windup
            self.integralFBy = 0.0
            self.integralFBz = 0.0
        
        # Apply proportional feedback
        gx += self.twoKp * halfex
        gy += self.twoKp * halfey
        gz += self.twoKp * halfez
    

        # Integrate rate of change of quaternion
        gx *= 0.5 * time_dt      # pre-multiply common factors
        gy *= 0.5 * time_dt
        gz *= 0.5 * time_dt
        qa = q0
        qb = q1
        qc = q2
        q0 += -qb * gx - qc * gy - q3 * gz
        q1 += qa * gx + qc * gz - q3 * gy
        q2 += qa * gy - qb * gz + q3 * gx
        q3 += qa * gz + qb * gy - qc * gx

        # Normalise quaternion
        normal = 1.0 / math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        q0 *= normal
        q1 *= normal
        q2 *= normal
        q3 *= normal

        self.q = (q0, q1, q2, q3)

    
    def update_no_mag(self, accel_xyz, gyro_xyz, time_dt):

        q0, q1, q2, q3 = self.q
        ax, ay, az = accel_xyz
        gx, gy, gz = gyro_xyz

        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        # Normalise accelerometer measurement
        normal = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax *= normal
        ay *= normal
        az *= normal

        # Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2
        halfvy = q0 * q1 + q2 * q3
        halfvz = q0 * q0 - 0.5 + q3 * q3

        # Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy)
        halfey = (az * halfvx - ax * halfvz)
        halfez = (ax * halfvy - ay * halfvx)

        # Compute and apply integral feedback if enabled
        if self.twoKi > 0.0 :
            self.integralFBx += self.twoKi * halfex * time_dt # integral error scaled by Ki
            self.integralFBy += self.twoKi * halfey * time_dt
            self.integralFBz += self.twoKi * halfez * time_dt
            gx += self.integralFBx  # apply integral feedback
            gy += self.integralFBy
            gz += self.integralFBz
        else:
            self.integralFBx = 0.0 # prevent integral windup
            self.integralFBy = 0.0
            self.integralFBz = 0.0

        # Apply proportional feedback
        gx += self.twoKp * halfex
        gy += self.twoKp * halfey
        gz += self.twoKp * halfez
    
        # Integrate rate of change of quaternion
        gx *= (0.5 * time_dt)      # pre-multiply common factors
        gy *= (0.5 * time_dt)
        gz *= (0.5 * time_dt)
        qa = q0
        qb = q1
        qc = q2
        q0 += (-qb * gx - qc * gy - q3 * gz)
        q1 += (qa * gx + qc * gz - q3 * gy)
        q2 += (qa * gy - qb * gz + q3 * gx)
        q3 += (qa * gz + qb * gy - qc * gx)

        # Normalise quaternion
        normal = 1.0 / math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        q0 *= normal
        q1 *= normal
        q2 *= normal
        q3 *= normal
        
        self.q = (q0, q1, q2, q3)

class QuaternionFusion2(object):
    '''
    Class provides sensor fusion allowing heading, pitch and roll to be extracted. This uses the Madgwick algorithm.
    The update method must be called peiodically. The calculations take 1.6mS on the Pyboard.
    '''
    
    def __init__(self):
        self.q = [1.0, 0.0, 0.0, 0.0]       # vector to hold quaternion
        GyroMeasError = radians(40)         # Original code indicates this leads to a 2 sec response time
        self.beta = sqrt(3.0 / 4.0) * GyroMeasError  # compute beta (see README)

    def update_imu(self, accel_xyz, gyro_xyz, compass_xyz, time_dt):
        self.update(accel_xyz, gyro_xyz, compass_xyz, time_dt)
        return (self.pitch, self.roll, self.yaw)
	
    
    #pitch(){ return -asin(2.0*q1*q3+2.0*q0*q2) }
    @property
    def pitch(self):
        return asin(2.0 * (self.q[0] * self.q[2] - self.q[1] * self.q[3]))
        #return -asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2]))
    
    #http://www.crazepony.com/book/wiki/hardware-algorithm.html        
    @property
    def roll(self):
        return atan2(2.0*(self.q[0] * self.q[1] + self.q[2] * self.q[3]), 
                1 - 2.0*(self.q[1] * self.q[1] + self.q[2] * self.q[2]))
        #return atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
        #    self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3])

    @property
    def yaw(self):
        return atan2( 2.0 * (self.q[0] * self.q[3] + self.q[1] * self.q[2]), 
                1 - 2.0*(self.q[2] * self.q[2] + self.q[3] * self.q[3]))
        #return atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
        #    self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3])
    
    def update_nomag(self, accel, gyro, time_dt):    # 3-tuples (x, y, z) for accel, gyro
        ax, ay, az = accel                  # Units G (but later normalised)
        gx, gy, gz = gyro                   
        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay
        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        
        q1 += qDot1 * time_dt
        q2 += qDot2 * time_dt
        q3 += qDot3 * time_dt
        q4 += qDot4 * time_dt

        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm

    def update(self, accel, gyro, mag, time_dt):     # 3-tuples (x, y, z) for accel, gyro and mag data
        mx, my, mz = mag                    # Units irrelevant (normalised)
        ax, ay, az = accel                  # Units irrelevant (normalised)
        gx, gy, gz = gyro                   # Units deg/s
        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return                          # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        # Gradient descent algorithm corrective step
        s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4)
             + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4)
             + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4)
              + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * time_dt
        q2 += qDot2 * time_dt
        q3 += qDot3 * time_dt
        q4 += qDot4 * time_dt
        
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm

