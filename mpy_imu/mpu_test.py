from i2c_mp import *
from mpu6050 import *

i2c = I2C_MP(1)
mpu = MPU6050(i2c)
mpu.init()
mpu.read()