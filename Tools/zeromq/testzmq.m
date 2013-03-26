clear all;
s1 = zmq('publish','matlab')
zmq('send',s1,'hello world!')
s2 = zmq('publish','debug')
zmq('send',s2,'hey-o')
s3 = zmq('subscribe','test')
zmq('receive',s3)
