import rpyc
from functools import partial
import timeit

conn = rpyc.connect("127.0.0.1", 5678)
#conn.root.init()

def rpyc_speed_test(service) :
    for i in range(5000):	
	service.update()

count = 10       
times = timeit.Timer(partial(rpyc_speed_test,  conn.root)).repeat(1, count)        
print count * 5000 / times[0] 


