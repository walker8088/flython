import Pyro4
from functools import partial
import timeit

nameserver = Pyro4.locateNS()
uri = nameserver.lookup("IMUSensors")
print uri
imu_server = Pyro4.Proxy(uri)
imu_server.init()

count = 1000       
times = timeit.Timer(partial(imu_server.update,)).repeat(1, count)        
print count / times[0] 


