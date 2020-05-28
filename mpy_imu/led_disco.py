
import pyb

def disco(ms):
    """
    cycles LEDs on pyboard for ms milliseconds
    """
    leds = [pyb.LED(i) for i in range(1,5)]
    for l in leds:
        l.off()
    now = pyb.millis()
    n = 0
    try:
       while pyb.millis() <= now + ms:
          n = (n + 1) % 4
          leds[n].toggle()
          pyb.delay(50)
    finally:
        for l in leds:
            l.off()