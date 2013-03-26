clear all;
s1 = zmq('publish','matlab')
zmq('send',s1,'hello world!')
s2 = zmq('publish','debug')
zmq('send',s2,'hey-o')
s3 = zmq('subscribe','test')
r = zmq('receive',s3)
char(r')
