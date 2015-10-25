from functools import partial
import timeit

import msgpackrpc

client = msgpackrpc.Client(msgpackrpc.Address("localhost", 18800))

def speed_test(client) :
	client.call("sum", 1, 2)
        client.sum(1,2)
count = 1000       
times = timeit.Timer(partial(speed_test, client)).repeat(1, count)        
print count / times[0] 


