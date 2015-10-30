
import math

# from http://www.pieter-jan.com/node/11
'''
#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
 
#define M_PI 3.14159265359	    
 
#define dt 0.01							// 10 ms sample rate!    
 
void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
} 
'''

def complementary_filter(gyro_xyz, accel_xyz, time_dt, roll, pitch):
	
	comp_value = 0.95
    roll += gyro_xyz[1] * time_dt
    pitch += gyro_xyz[0] * time_dt 

    #Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    foce = math.abs(accel_xyz[0]) + math.abs(accel_xyz[1])  + math.abs(accel_xyz[2])
    if (foce >= 512) and (foce <= 2014):
    	roll_acc = math.atan2f(accel_xyz[0], accel_xyz[2])
    	pitch_acc = math.atan2f(accel_xyz[1], accel_xyz[2])

    	roll = roll * comp_value + roll_acc * (1 - comp_value)
    	pitch = pitch * comp_value + pitch_acc * (1 - comp_value)

    return (roll, pitch) 
} 
