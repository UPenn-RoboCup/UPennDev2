dofile('include.lua')

-- Libraries
local simple_ipc = require 'simple_ipc'
local msgpack = require 'msgpack'
local carray = require 'carray'

-- Global vars
local actuator = {}
for i = 1, 20 do
  actuator[i] = i * math.pi
end

-- IPC channels
local camera_channel = simple_ipc.new_subscriber('camera')
local imu_channel = simple_ipc.new_subscriber('imu')

local actuator_channel = simple_ipc.new_subscriber('actuator')
local actuator_pub_channel = simple_ipc.new_publisher('actuator_cmd')
camera_channel.callback = function()
  local res = camera_channel:receive()
--  print('camera chanel ', #res)
end

imu_channel.callback = function()
  local res = imu_channel:receive()
  local imu_tbl = msgpack.unpack(res)
  --print('IMU:', unpack(imu_tbl) )
end

actuator_channel.callback = function()
  local res = actuator_channel:receive()
  local act_tbl = msgpack.unpack(res)
--  print("Actuators:", unpack(act_tbl))
end

local lidar_channel = simple_ipc.new_subscriber('lidar')
lidar_channel.callback = function()
  local ts, has_more = lidar_channel:receive();
  if not has_more then
    print("Bad lidar ts!")
    return
  end
  local ranges, has_more = lidar_channel:receive();
  lidar_ts = tonumber(ts);
  lidar_ranges = carray.float( ranges );
  print(lidar_ts," Lidar: ", #lidar_ranges)
end

local wait_channels = {imu_channel, camera_channel, actuator_channel, lidar_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels )

local channel_timeout = 30
while true do
  channel_poll:poll(channel_timeout)
  --local str = msgpack.pack(actuator)
  --print(#str)
  --actuator_pub_channel:send( msgpack.pack(actuator) )
end
