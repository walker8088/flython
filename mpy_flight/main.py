
import pyb
from mpu6050 import MPU6050

# def angle_to_freq(a): return a*1000/90+500
# def freq_to_angle(f): return f*90/1000-45

#accel = pyb.Accel()
imu = MPU6050()
usb = pyb.USB_VCP()  # for MPUTeapot.py

# servos
x_sr = pyb.Servo(3)
y_sr = pyb.Servo(4)

# rc receiver
t2 = pyb.Timer(2, prescaler=83, period=0x0fffffff)
x_pn = pyb.Pin.board.X1
y_pn = pyb.Pin.board.X2
x_ch = t2.channel(1, pyb.Timer.IC, pin=x_pn, polarity=pyb.Timer.BOTH)
y_ch = t2.channel(2, pyb.Timer.IC, pin=y_pn, polarity=pyb.Timer.BOTH)
x_start = x_width = 0
y_start = y_width = 0
x_trim = y_trim = -135  # -45
x_reverse = 1
y_reverse = -1

def x_cb(tim):
  global x_start, x_width
  if x_pn.value(): x_start = x_ch.capture()
  else: x_width = x_ch.capture() - x_start & 0x0fffffff

def y_cb(tim):
  global y_start, y_width
  if y_pn.value(): y_start = y_ch.capture()
  else: y_width = y_ch.capture() - y_start & 0x0fffffff

x_ch.callback(x_cb)
y_ch.callback(y_cb)

while True:
  start = pyb.millis()
  #x, y, z = accel.filtered_xyz()
  x, y, z = imu.get_xyz()
  ax = (x_width * 0.09 + x_trim) * x_reverse - x
  ay = (y_width * 0.09 + y_trim) * y_reverse - y
  #print('x:%s, y:%s, z:%s' % (ax, ay, z))
  usb.write('%s,%s,%s' % (ax, ay, z))  # for MPUTeapot.py
  x_sr.angle(ax)
  y_sr.angle(ay)
  elapsed = pyb.elapsed_millis(start)
  if elapsed < 50: pyb.delay(50 - elapsed)