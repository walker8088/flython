
import pyb

from struct import unpack as unp
from math import atan2, sqrt, pow, pi

class MPU6050():
  mpu_addr = 0x68  # address of MPU6050                                        
  FREQ = 30
  _I2Cerror = "I2C communication failure"
  def __init__(self, side=1, disable_interrupts=True):
    self._timeout = 10
    self.disable_interrupts = False
    self._mpu_i2c = pyb.I2C(side, pyb.I2C.MASTER)
    self.chip_id = int(unp('>h', self._read(1, 0x75))[0])
    self.disable_interrupts = disable_interrupts
    # wake it up
    self._write(0x00, 0x6B)  # wake up
    self._write(0x01, 0x1A)  # low pass filter samples, 1khz sample rate
    # self._write(0x40, 0x6B)  # sleep
    self._sr = self.sample_rate(30)
    self._ar = self.accel_range(1)
    self._gr = self.gyro_range(0)
    # calibrate
    self.gx_offs = self.gy_offs = self.gz_offs = 0
    self.calibrate()
    # filtered xyz
    self.gx = self.gy = self.gz = 0

  def _read(self, count, memaddr):
    irq_state = True
    if self.disable_interrupts:
      irq_state = pyb.disable_irq()
    result = self._mpu_i2c.mem_read(count,
                                    self.mpu_addr,
                                    memaddr,
                                    timeout=self._timeout)
    pyb.enable_irq(irq_state)
    return result

  def _write(self, data, memaddr):
    irq_state = True
    if self.disable_interrupts:
      irq_state = pyb.disable_irq()
    result = self._mpu_i2c.mem_write(data,
                                     self.mpu_addr,
                                     memaddr,
                                     timeout=self._timeout)
    pyb.enable_irq(irq_state)
    return result

  def sample_rate(self, sample_rate):
    '''
    sample rate FREQ = Gyro sample rate / (sample_div + 1)
    1kHz / (div + 1) = FREQ
    reg_value = 1khz/FREQ - 1
    '''
    try:
      rate_div = int(1000 / sample_rate - 1)
      if rate_div > 255:
        rate_div = 255
      self._write(rate_div, 0x19)
      rate = int(unp('<H', self._read(1, 0x19))[0]+1)
    except OSError:
      rate = None
    return rate

  def accel_range(self, accel_range):
    '''
    Returns the accelerometer range or sets it to the passed arg.
    Pass:               0   1   2   3
    for range +/-:      2   4   8   16  g
    '''
    try:
      ar = (0x00, 0x08, 0x10, 0x18)
      try:
        self._write(ar[accel_range], 0x1C)
      except IndexError:
        print('accel_range can only be 0, 1, 2 or 3')
      ari = int(unp('<H', self._read(1, 0x1C))[0]/8)
    except OSError:
      ari = None
    return ari

  def gyro_range(self, gyro_range):
    '''
    Returns the gyroscope range or sets it to the passed arg.
    Pass:               0   1   2    3
    for range +/-:      250 500 1000 2000  degrees/second
    '''
    try:
      gr = (0x00, 0x08, 0x10, 0x18)
      try:
        self._write(gr[gyro_range], 0x1B)
      except IndexError:
        print('gyro_range can only be 0, 1, 2 or 3')
      gri = int(unp('<H', self._read(1, 0x1B))[0]/8)
    except OSError:
      gri = None...