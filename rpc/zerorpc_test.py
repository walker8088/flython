from functools import partial
import timeit

import zerorpc

c = zerorpc.Client()
c.connect("tcp://127.0.0.1:4242")

count = 10000       
times = timeit.Timer(partial(c.hello,"me")).repeat(1, count)        
print count / times[0] 


