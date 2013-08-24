-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'../include.lua'

-- Libraries
require'unix'
require'vcm'
local torch      = require'torch'
torch.Tensor     = torch.DoubleTensor
local util       = require'util'
local jpeg       = require'jpeg'
local png        = require'png'
local Body       = require'Body'
local util       = require'util'
local mp         = require'msgpack'
local carray     = require'carray'
local simple_ipc = require'simple_ipc'
local udp        = require'udp'
local udp_port   = 43288
local udp_target = '192.168.123.23'

-- Sending the mesh messages on zmq or UDP
local mesh_pub_ch = simple_ipc.new_publisher'mesh'
local mesh_udp_ch = udp.new_sender( udp_target, udp_port )

-- Robot-wide used Mesh images parameters
local chest = {}
-- Desired panning resolution
chest.res       = vcm.get_chest_lidar_mesh_resolution()
-- Panning endpoints
chest.start     = vcm.get_chest_lidar_endpoints()[1]
chest.stop      = vcm.get_chest_lidar_endpoints()[2]
-- TODO: Be able to resize these
chest.mesh_byte = torch.ByteTensor(  unpack(chest.res) ):zero()
chest.mesh      = torch.FloatTensor( unpack(chest.res) ):zero()
chest.mesh_adj  = torch.FloatTensor( unpack(chest.res) ):zero()
-- Index Offset for copying using carray
-- TODO: Get 1081 as a Body parameter for the size of the lidar
chest.offset    = math.floor( (1081-chest.res[2])/2 )
chest.lidar_ch  = simple_ipc.new_subscriber'chest_lidar'
chest.meta      = {}
chest.meta.name = 'chest'

-- Convert a pan angle to a column of the chest mesh image
local function pan_to_column( rad )
  rad = math.max( math.min(rad, chest.stop), chest.start )
  local ratio = (rad-chest.start)/(chest.stop-chest.start)
  local col = math.floor(ratio*chest.res[2]+.5)
  -- Do not return a column number if out of bounds
  if col>=1 and col<=chest.res[1] then return col end
end

-- type is head or chest table
local function stream_mesh(type)

  -- Grab the parameters
  local stream_mode = vcm.get_chest_lidar_mesh_stream()
  -- No streaming right now
  if stream_mode==0 then return end
  
  local my_range = vcm.get_chest_lidar_mesh_range()

  -- Safety check
  if my_range[1]>=my_range[2] then return end
  
  -- Associate metadata with this depth image
  local metadata = type.meta
  metadata.t     = unix.time()
  metadata.res   = type.res

  -- Enhance the dynamic range of the mesh image
  local adjusted_range = type.mesh_adj
  adjusted_range:copy(type.mesh):add( -my_range[1] )
  adjusted_range:mul( 255/(my_range[2]-my_range[1]) )
    
  -- Ensure that we are between 0 and 255
  adjusted_range[torch.lt(adjusted_range,0)] = 0
  adjusted_range[torch.gt(adjusted_range,255)] = 255
  type.mesh_byte:copy( adjusted_range )
  
  -- Compress the mesh
  local payload = nil
  if stream_mode==3 then
    payload = png.compress(
    type.mesh_byte:storage():pointer(),
    type.mesh_byte:size(2),
    type.mesh_byte:size(1),
    1 )
  elseif stream_mode==2 then
    jpeg.set_quality( my_range[3] )
    payload = jpeg.compress_gray(
    type.mesh_byte:storage():pointer(),
    type.mesh_byte:size(2),
    type.mesh_byte:size(1) )
  end
  
  -- Perform the sending
  local meta = mp.pack(metadata)
  --mesh_pub_ch:send( {meta, payload} )
  local pkt = mp.pack(metadata)..payload
  local ret, err = mesh_udp_ch:send( pkt, #pkt )
  print(err or string.format('Sent a %g kB chest mesh packet!', ret / 1024))
  
end

------------------------------
-- Lidar Callback functions
local function chest_callback()
  
  local meta, has_more = chest.lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_chest_lidar()
  -- Insert into the correct column
  local col = pan_to_column(metadata.pangle)
  -- Only if a valid column is returned
  if col then
    -- Copy to the torch object for fast modification
    ranges:tensor( chest.mesh:select(1,col), chest.res[2], chest.offset )
  end
  -- Stream the current mesh
  stream_mesh(chest)
end

local function head_callback()
  local meta, has_more = head_lidar_ch:receive()
end

------------------------------
-- Polling with zeromq
local wait_channels = {}
if head_lidar_ch  then
  head_lidar_ch.callback = head_callback
  table.insert( wait_channels, head_lidar_ch )
end
if chest.lidar_ch then
  chest.lidar_ch.callback = chest_callback
  table.insert( wait_channels, chest.lidar_ch )
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