
import math
from pyrr import *

time_dt = 1.0 #temp value for guess

Kp_ROLLPITCH = 0.02
Ki_ROLLPITCH = 0.00002

Kp_YAW = 1.8
Ki_YAW = 0.00002

Gyro_Gain_X = 0.0305
Gyro_Gain_Y = 0.0305
Gyro_Gain_Z = 0.0305

def mag_heading(mag_vector, roll, pitch, mag_declination):       
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)

        #Tilt compensated Magnetic field X component:
        head_x = mag_vector.x * cos_pitch + mag_vector.y * sin_roll * sin_pitch + mag_vector.z * cos_roll * sin_pitch
        #Tilt compensated Magnetic field Y component:
        head_y = mag_vector.y * cos_roll - mag_vector.z * sin_roll
        #Magnetic heading
        heading = math.atan2(-head_y, head_x)

        heading += mag_declination
        if (heading > math.pi):    #// Angle normalization (-180 deg, 180 deg)
                heading -= (2.0 * math.pi)
        elif (heading < -math.pi):
                heading += (2.0 * math.pi)

        return heading

        #// Optimization for external DCM use. Calculate normalized components
        #heading_X = cos(heading);
        #heading_Y = sin(heading);

class DCMFusion(object):
    def __init__(self, mag_declination = 0):
    
        self.mag_declination = mag_declination
        
        self.accel_vector = Vector3()
        self.gyro_vector = Vector3()
        self.omege_vector = Vector3()
        self.dcm_matrix = Matrix33([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
                ])
       
        self.omega_p = Vector3([0,0,0]) #omega Proportional correction
        self.omega_i = Vector3([0,0,0]) #
        
    def euler_angles(self):
        
        roll = math.atan2(self.dcm_matrix[2][1], self.dcm_matrix[2][2])
        pitch = -math.asin(self.dcm_matrix[2][0])
        yaw = math.atan2(self.dcm_matrix[1][0], self.dcm_matrix[0][0])
        
        return (roll, pitch, yaw)    
        
    def update(self, accel_xyz, gyro_xyz, omega_xyz, time_dt):
    
        self.update_dcm(Vector3(accel_xyz), Vector3(gyro_xyz), Vector3(omega_xyz), time_dt)
        self.normalize()      
        roll, pitch, _ = self.euler_angles()
        heading = mag_heading(self.omege_vector, roll, pitch, self.mag_declination)
        self.drift_correction(heading)
       
        return self.euler_angles()
        
    def update_dcm(self, accel_vector, gyro_vector, omega_vector, time_dt):
        self.accel_vector = accel_vector
        self.gyro_vector = gyro_vector #gyro x roll,y pitch,z yaw
        
        #Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
        #Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
        self.omega_vector =  gyro_vector + self.omega_i + self.omega_p
        '''
        Update_Matrix[0][0] = 0;
        Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; //-z
        Update_Matrix[0][2] = G_Dt * Omega_Vector[1]; //y
        Update_Matrix[1][0] = G_Dt * Omega_Vector[2]; //z
        Update_Matrix[1][1] = 0;
        Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; //-x
        Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; //-y
        Update_Matrix[2][1] = G_Dt * Omega_Vector[0]; //x
        Update_Matrix[2][2] = 0;

        Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

        for (int x = 0; x < 3; x++) //Matrix Addition (update)
        {
        for (int y = 0; y < 3; y++)
        {
                DCM_Matrix[x][y] += Temporary_Matrix[x][y];
        }
        }
        }
        '''
        self.dcm_matrix += self.dcm_matrix * Matrix33([
                [0, -time_dt * self.omega_vector[2], time_dt * self.omega_vector[1]], #[0, -z, y]
                [time_dt * self.omega_vector[2], 0, -time_dt * self.omega_vector[0]], #[z, 0, -x] 
                [-time_dt * omega_vector[1], time_dt * omega_vector[0], 0]            #[-y, x, 0] 
                ])
        
        
    def normalize(self):
         
        #error = -Vector_Dot_Product(&dcm_matrix[0][0], &dcm_matrix[1][0]) * .5; //eq.19
        error = -Vector3.dot(self.dcm_matrix[0], self.dcm_matrix[1]) * 0.5 #eq.19
        
        #Vector_Scale(&temporary[0][0], &dcm_matrix[1][0], error); //eq.19
        #Vector_Scale(&temporary[1][0], &dcm_matrix[0][0], error); //eq.19
        #Vector_Add(&temporary[0][0], &temporary[0][0], &dcm_matrix[0][0]);//eq.19
        #Vector_Add(&temporary[1][0], &temporary[1][0], &dcm_matrix[1][0]);//eq.19
        temp0 = self.dcm_matrix[1] * error + self.dcm_matrix[0] #eq.19
        temp1 = self.dcm_matrix[0] * error + self.dcm_matrix[1] #eq.19
        
        #Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20
        temp2 = Vector3.cross(temp0,temp1) #eq.20
        
        #renorm = Vector_Dot_Product(&temporary[0][0], &temporary[0][0]);
        renorm = Vector3.dot(temp0, temp0) 
        
        if (0.64 < renorm < 1.5625):
                renorm = 0.5 * (3 - renorm)   #eq.21
        elif (0.01 < renorm < 100.0):
                renorm = 1.0 / math.sqrt(renorm)
        else:
                raise Exception("error")
        
        #Vector_Scale(&dcm_matrix[0][0], &temporary[0][0], renorm);
        self.dcm_matrix[0] = temp0 * renorm
        
        #renorm = 0.5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
        #Vector_Scale(&dcm_matrix[1][0], &temporary[1][0], renorm);
        renorm = 0.5 * (3 - Vector3.dot(temp1,temp1))  #eq.21
        self.dcm_matrix[1] = temp1 * renorm

        #renorm = .5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
        #Vector_Scale(&dcm_matrix[2][0], &temporary[2][0], renorm);
        renorm = 0.5 * (3 - Vector3.dot(temp2, temp2))  #eq.21
        self.dcm_matrix[2] = temp2 * renorm
        
    def drift_correction(self, heading):
        
        mag_heading_x = math.cos(heading)
        mag_heading_y = math.sin(heading)
        
        accel_magnitude = self.accel_vector.length
        accel_magnitude = accel_magnitude / GRAVITY  #Scale to gravity.
        
        #Dynamic weighting of accelerometer info (reliability filter)
        #Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
        #accel_weight = constrain(1 - 2 * abs(1 - accel_magnitude), 0, 1); //
        accel_weight = math.cconstrain(1 - 2 * math.abs(1 - accel_magnitude), 0, 1)
        
        #Vector_Cross_Product(&error_roll_pitch[0], &accel_vector[0], &dcm_matrix[2][0]); //adjust the ground of reference
        error_roll_pitch  = Vector3.cross(self.accel_vector,self.dcm_matrix[2])
        
        #Vector_Scale(&omega_p[0], &error_roll_pitch[0], Kp_ROLLPITCH * accel_weight);
        self.omega_p = error_roll_pitch  * Kp_ROLLPITCH * accel_weight
        
        #Vector_Add(omega_i, omega_i, scaled_omega_i);
        self.omega_i += scaled_omega_i
        
        #*****YAW***************
        #We make the gyro YAW drift correction based on compass magnetic heading
        error_course = (self.dcm_matrix[0][0] * mag_heading_y) - (self.dcm_matrix[1][0] * mag_heading_x) #Calculating YAW error
        
        #Vector_Scale(error_yaw, &dcm_matrix[2][0], error_course); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
        error_yaw = self.dcm_matrix[2] * error_course
         
        #Vector_Scale(&scaled_omega_p[0], &error_yaw[0], Kp_YAW); //.01proportional of YAW.
        #Vector_Add(omega_p, omega_p, scaled_omega_p); //Adding  Proportional.
        self.omega_p +=error_yaw * Kp_YAW
        
        #Vector_Scale(&scaled_omega_i[0], &error_yaw[0], Ki_YAW); //.00001Integrator
        #Vector_Add(omega_i, omega_i, scaled_omega_i); //adding integrator to the omega_i
        self.omega_i += error_yaw * Ki_YAW
        
