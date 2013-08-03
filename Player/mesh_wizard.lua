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
-- TODO: Ajust the downsample, since we may not care about 30 meters away, but just 2 meters
-- 30 meters is the max range of the hokuyo and the max float we will see
local chest_range     = vcm.get_chest_lidar_mesh_range()
local chest_res       = vcm.get_chest_lidar_mesh_resolution()
local chest_start     = vcm.get_chest_lidar_endpoints()[1]
local chest_stop      = vcm.get_chest_lidar_endpoints()[2]
-- TODO: Get 1081 as a Body parameter for the size of the lidar
local chest_offset    = math.floor( (1081-chest_res[2])/2 )
local chest_mesh_byte = torch.ByteTensor( unpack(chest_res) ):zero()
local chest_mesh      = torch.FloatTensor( unpack(chest_res) ):zero()

-- Convert a pan angle to a column of the mesh image
local function pan_to_column( rad )
  rad = math.max( math.min(rad, chest_stop), chest_start )
  local ratio = (rad-chest_start)/(chest_stop-chest_start)
  ratio = math.min(ratio,1)
  return math.floor(math.max(ratio*chest_res[1],1)+.5)
end

------------------------------
-- Callback functions
local function chest_callback()
  local t = unix.time()
	local meta, has_more = chest_lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_chest_lidar()
  -- Insert into the correct column
  local col = pan_to_column(metadata.pangle)
  local chest_slice = chest_mesh:select(1,col)
  
  -- Copy to the torch object for fast modification
  ranges:tensor( chest_slice, chest_res[2], chest_offset )
  
  -- Check if we wish to save and send to the user
  local save_request = vcm.get_chest_lidar_mesh_save()
  if save_request==1 then
    -- Remove the save request
    vcm.set_chest_lidar_mesh_save(0)
    -- Enhance the dynamic range of the mesh image
    local adjusted_range = torch.add( chest_mesh, -chest_range[1] )
    adjusted_range:mul( 255/(chest_range[2]-chest_range[1]) )
    
    -- Ensure that we are between 0 and 255
    adjusted_range[torch.lt(adjusted_range,0)] = 0
    adjusted_range[torch.gt(adjusted_range,255)] = 255
    
    -- Compress to JPEG for sending over the wire
    -- TODO: We may not wish to JPEG the image in some cases
    chest_mesh_byte:copy( adjusted_range )
    local jdepth = jpeg.compress_gray(
      chest_mesh_byte:storage():pointer(),
      chest_res[2],chest_res[1] )
      
    -- TODO: Associate metadata with this depth image?
  end
  
end

local function head_callback()
  --print('Head!')
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