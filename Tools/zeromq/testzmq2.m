clear all;
s3 = zmq('subscribe',5555)
r = [];
while numel(r)<1
  [r data] = zmq('poll',100);
end
