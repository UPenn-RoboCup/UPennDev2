local simple_ipc = require 'simple_ipc'
local mp = require 'msgpack'
local test_channel = simple_ipc.new_publisher('test'); --ipc
local imu = {Ax=0,Ay=0,Az=1,Wx=0,Wy=0,Wz=1}
while true do
  imu.Wz = math.random(1)
	test_channel:send( mp.pack(imu) )
  os.execute ('sleep .5')
end
