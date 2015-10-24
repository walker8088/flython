import rpyc
from functools import partial
import timeit
import numpy as np

conn = rpyc.connect("192.168.0.108", 5678)
conn.root.init()

def rpyc_speed_test(service) :
        service.update()
count = 1000       
times = timeit.Timer(partial(rpyc_speed_test,  conn.root)).repeat(1, count)        
print count / times[0] 


