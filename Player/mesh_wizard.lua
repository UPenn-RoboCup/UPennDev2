-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'../include.lua'

-- Libraries
require 'unix'
local torch      = require'torch'
torch.Tensor     = torch.DoubleTensor
local simple_ipc = require'simple_ipc'
local carray     = require'carray'
local jpeg       = require'jpeg'
local mp         = require'msgpack'
local Body       = require'Body'
local util       = require'util'

-- Mesh images
local chest_mesh_byte  = torch.ByteTensor( 3, 1081 ):zero()
local chest_mesh  = torch.FloatTensor( 3, 1081 ):zero()

-- Convert a pan angle to a column of the mesh image
local function pan_to_column( pan )
  return 1
end

------------------------------
-- Callback functions
local function chest_callback()
	local meta, has_more = chest_lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_chest_lidar()
  -- Insert into the correct column
  local col = pan_to_column(metadata.pangle)
  -- Copy to the torch object for fast modification
  ranges:tensor( chest_mesh:select(1,col) )
  
  print('CHEST | Inserting Column '..col..'...')  
  -- TODO: Scale into 0-255 for jpeg compression when desired
  
end

local function head_callback()
	local meta, has_more = head_lidar_ch:receive()
  local head_angle = Body.get_head_command_position(1)
end
------------------------------

------------------------------
-- Polling with zeromq
head_lidar_ch  = simple_ipc.new_subscriber'head_lidar'
chest_lidar_ch = simple_ipc.new_subscriber'chest_lidar'

-- Setup the callbacks for ZMQ
local wait_channels = {}
if head_lidar_ch  then
  head_lidar_ch.callback = head_callback
  table.insert( wait_channels, head_lidar_ch )
end
if chest_lidar_ch then
  chest_lidar_ch.callback = chest_callback
  table.insert( wait_channels, chest_lidar_ch )
end
local channel_polls = simple_ipc.wait_on_channels( wait_channels )

-- Begin polling
channel_polls:start()
--[[
local channel_timeout = 100;
while true do
  local npoll = channel_polls:poll(channel_timeout)
  if npoll<1 then print'timeout' end
end
--]]