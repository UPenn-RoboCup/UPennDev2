clear all;
s3 = zmq('subscribe',5555)
idx = [];
while numel(idx)<1
  [data idx] = zmq('poll',100);
end
msgpack('unpack', data{1})
idx
