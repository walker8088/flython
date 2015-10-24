"""
MS5611 driver code is placed under the BSD license.
Copyright (c) 2014, Emlid Limited, www.emlid.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.
	* Neither the name of the Emlid Limited nor the names of its contributors
	may be used to endorse or promote products derived from this software
	without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import time
import math

MS5611_ADDRESS_CSB_LOW  = 0x76
MS5611_ADDRESS_CSB_HIGH = 0x77
MS5611_DEFAULT_ADDRESS  = 0x77

MS5611_RA_ADC           = 0x00
MS5611_RA_RESET         = 0x1E

MS5611_RA_C0            = 0xA0
MS5611_RA_C1            = 0xA2
MS5611_RA_C2            = 0xA4
MS5611_RA_C3            = 0xA6
MS5611_RA_C4            = 0xA8
MS5611_RA_C5            = 0xAA
MS5611_RA_C6            = 0xAC
MS5611_RA_C7            = 0xAE

MS5611_RA_D1_OSR_256    = 0x40
MS5611_RA_D1_OSR_512    = 0x42
MS5611_RA_D1_OSR_1024   = 0x44
MS5611_RA_D1_OSR_2048   = 0x46
MS5611_RA_D1_OSR_4096   = 0x48

MS5611_RA_D2_OSR_256    = 0x50
MS5611_RA_D2_OSR_512    = 0x52
MS5611_RA_D2_OSR_1024   = 0x54
MS5611_RA_D2_OSR_2048   = 0x56
MS5611_RA_D2_OSR_4096   = 0x58

class MS5611 :
	def __init__(self, i2c, address = 0x77):
		
                self.i2c = i2c
		self.address = address
		
                self.C1 = 0
		self.C2 = 0
		self.C3 = 0
		self.C4 = 0
		self.C5 = 0
		self.C6 = 0
		self.D1 = 0
		self.D2 = 0
                
		self.temp = 0.0 # Calculated temperature
		self.press = 0.0 # Calculated Pressure

	def init(self):
		## The MS6511 Sensor stores 6 values in the EPROM memory that we need in order to calculate the actual temperature and pressure
		## These values are calculated/stored at the factory when the sensor is calibrated.
		C1 = self.i2c.read_reg_block(self.address, MS5611_RA_C1) #Pressure Sensitivity
		C2 = self.i2c.read_reg_block(self.address, MS5611_RA_C2) #Pressure Offset
		C3 = self.i2c.read_reg_block(self.address, MS5611_RA_C3) #Temperature coefficient of pressure sensitivity
		C4 = self.i2c.read_reg_block(self.address, MS5611_RA_C4) #Temperature coefficient of pressure offset
		C5 = self.i2c.read_reg_block(self.address, MS5611_RA_C5) #Reference temperature
		C6 = self.i2c.read_reg_block(self.address, MS5611_RA_C6) #Temperature coefficient of the temperature

		## Again here we are converting the 2 8bit packages into a single decimal
                self.C1 = C1[0] * 256 + C1[1]
		self.C2 = C2[0] * 256 + C2[1]
		self.C3 = C3[0] * 256 + C3[1]
		self.C4 = C4[0] * 256 + C4[1]
		self.C5 = C5[0] * 256 + C5[1]
		self.C6 = C6[0] * 256 + C6[1]

		self.update()

	def refreshPressure(self, OSR = MS5611_RA_D1_OSR_4096):
		self.i2c.write_byte(self.address, OSR)

	def refreshTemperature(self, OSR = MS5611_RA_D2_OSR_4096):
		self.i2c.write_byte(self.address, OSR)

	def readPressure(self):
		D1 = self.i2c.read_reg_block(self.address, MS5611_RA_ADC)
		self.D1 = D1[0] * 65536.0 + D1[1] * 256 + D1[2]

	def readTemperature(self):
		D2 = self.i2c.read_reg_block(self.address, MS5611_RA_ADC)
		self.D2 = D2[0] * 65536.0 + D2[1] * 256 + D2[2]

	def calculatePressureAndTemperature(self):
        
		dT = self.D2 - self.C5 * 2**8
		TEMP = 2000 + dT * self.C6 / 2**23

		OFF = self.C2 * 2**16 + (self.C4 * dT) / 2**7
		SENS = self.C1 * 2**15 + (self.C3 * dT) / 2**8

		if (TEMP >= 2000):
			T2 = 0
			OFF2 = 0
			SENS2 = 0
		elif (TEMP < 2000):
			T2 = dT * dT / 2**31
			OFF2 = 5 * ((TEMP - 2000) ** 2) / 2
			SENS2 = OFF2 / 2
		elif (TEMP < -1500):
			OFF2 = OFF2 + 7 * ((TEMP + 1500) ** 2)
			SENS2 = SENS2 + 11 * (TEMP + 1500) ** 2 / 2

		TEMP = TEMP - T2
		OFF = OFF - OFF2
		SENS = SENS - SENS2
		PRES = (self.D1 * SENS / 2**21 - OFF) / 2**15

                self.temp  =  TEMP / 100 # Temperature updated
                self.press = PRES / 100 # Pressure updated
		
	def update(self):
		self.refreshPressure()
		time.sleep(0.01) # Waiting for pressure data ready
		self.readPressure()

		self.refreshTemperature()
		time.sleep(0.01) # Waiting for temperature data ready
		self.readTemperature()

		self.calculatePressureAndTemperature()
		
		return (self.press, self.temp) 
    	

def pt2altitude(press, temp):
        #return ((math.pow((101325.0 / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065
	return 

if __name__=='__main__':
    
    import sys	
    sys.path.append('..')

    from bus import I2C

    i2c = I2C(1)
    baro = MS5611(i2c)
    baro.init()

    presss = []
    temps = []	
    for i in range(100) :
	press, temp = baro.update()
	presss.append(press)
	temps.append(temp)
   	time.sleep(0.01)

    press_base = sum(presss)/len(presss) 
    temp_base = sum(temps)/len(temps)
    altitude_base = pt2altitude(press_base, temp_base)
    print press_base, temp_base, altitude_base

    while(True):
        baro.update()

        print "Temperature(C): %.6f" % (baro.temp), "Pressure(millibar): %.6f" % (baro.press)
	scaling = baro.press / press_base
        altitude_diff = 153.8462 * (temp_base + 273.15) * (1.0 - math.exp(0.190259 * math.log(scaling)))
	print "%.3f" % altitude_diff

        time.sleep(0.1)
