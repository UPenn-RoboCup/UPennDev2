dofile('include.lua')

-- Libraries
local simple_ipc = require 'simple_ipc'
local msgpack = require 'msgpack'
local carray = require 'carray'
local Octomap = require'Octomap'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local libLaser = require 'libLaser'

-- Global vars
require 'unix'
require 'Params'
local actuator_positions = {};
local actuator_commands = {}
actuator_commands = carray.double(#Params.jointNames);
for i = 1,#Params.jointNames do
  actuator_commands[i] = 0;
  actuator_positions[i] = 0;
end

-- IPC channels
local camera_channel = simple_ipc.new_subscriber('camera')
camera_channel.callback = function()
  local res = camera_channel:receive()
  print('camera chanel ', #res)
end

local lidar_channel = simple_ipc.new_subscriber('lidar')
lidar_channel.callback = function()
  local ts, has_more = lidar_channel:receive();
  if not has_more then
    print("Bad lidar ts!")
    return
  end
  local ranges_str, has_more = lidar_channel:receive();
  local lidar_ts = tonumber(ts);
  local ranges_f = carray.double( ranges_str );
  
  -- LIDAR debug
  print(lidar_ts," Lidar: ", #ranges_f)

  -- Use a silly element-by-element copy
  local ranges = torch.Tensor( 1081 )
  for i=1,#ranges_f do
    ranges[i] = ranges_f[i]
  end
  -- Use pointers for "efficiency"
--[[
  local ranges_s = torch.FloatStorage( 1081, ranges_f:pointer() )
  local ranges = torch.FloatTensor( ranges_s );
--]]
  
  -- TODO: Use for slam as well
  libLaser.ranges2xyz(ranges,0,0,0)
  --Octomap.add_scan( libLaser.points_xyz )
  
  -- Change the lidar head to scan
  local pitch = 10*math.cos( ts ) + 20
  actuator_commands[2] = pitch*math.pi/180
end

local wait_channels = {camera_channel, lidar_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels )

--local channel_timeout = 30
local channel_timeout = -1
local t0 = unix.time()
local t_last = t0;
while true do
  local n_poll = channel_poll:poll(channel_timeout)
  -- Send actuator commands after each update?
  --local ret = actuator_pub_channel:send( tostring(actuator_commands) )
  local t = unix.time()
  local fps = 1/(t-t_last)
  t_last = t;
  local debug_msg = string.format(
  "Updating at %.3f FPS",
  fps
  )
  print( debug_msg )
end
