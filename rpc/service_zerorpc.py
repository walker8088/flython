import zerorpc

class HelloRPC(object):
    def hello(self, name):
        return '0'

s = zerorpc.Server(HelloRPC())
s.bind("tcp://0.0.0.0:4242")
s.run()
