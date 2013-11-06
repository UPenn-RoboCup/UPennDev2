dofile'../../include.lua'
local simple_ipc = require 'simple_ipc'
require'unix'
local mp = require 'msgpack'
--local test_channel = simple_ipc.new_publisher('test'); --ipc
local test_channel = simple_ipc.new_publisher(55555); --tcp
local imu = {Ax=0,Ay=0,Az=1,Wx=0,Wy=0,Wz=1}
while true do
  imu.Wz = math.random(1)
  test_channel:send( mp.pack(imu) )
print'hi'
  unix.usleep(5e5)
end
