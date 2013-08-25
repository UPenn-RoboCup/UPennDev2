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
local udp_port   = 54345
local udp_target = 'localhost'
--local udp_target = '192.168.123.23'
jpeg.set_quality( 95 )

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
chest.lidarangles  = torch.DoubleTensor( chest.res[1] ):zero()
-- Index Offset for copying using carray
-- TODO: Get 1081 as a Body parameter for the size of the lidar
chest.offset    = math.floor( (1081-chest.res[2])/2 )
chest.lidar_ch  = simple_ipc.new_subscriber'chest_lidar'
chest.meta      = {}
chest.meta.name = 'chest'
chest.meta.invert = true
local head = {}
-- Desired panning resolution
head.res       = vcm.get_head_lidar_mesh_resolution()
-- Tilting endpoints
head.start     = vcm.get_head_lidar_endpoints()[1]
head.stop      = vcm.get_head_lidar_endpoints()[2]
-- TODO: Be able to resize these
head.mesh_byte = torch.ByteTensor(  unpack(head.res) ):zero()
head.mesh      = torch.FloatTensor( unpack(head.res) ):zero()
head.mesh_adj  = torch.FloatTensor( unpack(head.res) ):zero()
head.lidarangles  = torch.DoubleTensor( head.res[1] ):zero()
-- Index Offset for copying using carray
head.offset    = math.floor( (1081-head.res[1])/2 )
head.lidar_ch  = simple_ipc.new_subscriber'head_lidar'
head.meta      = {}
head.meta.name = 'head'
head.meta.invert = false

-- Convert a pan angle to a column of the chest mesh image
local function pan_to_column( rad )
  rad = math.max( math.min(rad, chest.stop), chest.start )
  local ratio = (rad-chest.start)/(chest.stop-chest.start)
  local col = math.floor(ratio*chest.res[2]+.5)
  -- Do not return a column number if out of bounds
  if col<1 or col>chest.res[1] then return end
  return col
end

-- Convert a head tilt angle to a row of the head mesh image
local function tilt_to_row( rad )
  rad = math.max( math.min(rad, head.stop), head.start )
  local ratio = (rad-head.start)/(head.stop-head.start)
  local row = math.floor(ratio*head.res[1]+.5)
  -- Do not return a row number if out of bounds
  if row<1 or row>head.res[2] then return end
  return row
end

-- type is head or chest table
local function stream_mesh(type)
  -- Network streaming settings
  local net_settings = vcm['get_'..type.meta.name..'_lidar_net']()
  print(type.meta.name,net_settings,'net_settings')
  -- Streaming
  if net_settings[1]==0 then return end
  if net_settings[1]==1 then
    net_settings[1] = 0
    vcm['set_'..type.meta.name..'_lidar_net'](net_settings)
  end
  -- Sensitivity range in meters
  local my_range = vcm['get_'..type.meta.name..'_lidar_mesh_range']()
  -- Safety check
  if my_range[1]>=my_range[2] then return end
  
  -- Associate metadata with this depth image
  local metadata = type.meta
  metadata.t     = unix.time()
  metadata.res   = type.res
  metadata.lidarangles = type.lidarangles
  metadata.lidarrange = type.res[2]-type.res[1]
  metadata.range0 = type.start
  metadata.range1 = type.stop

  -- Enhance the dynamic range of the mesh image
  local adjusted_range = type.mesh_adj
  adjusted_range:copy(type.mesh):add( -my_range[1] )
  adjusted_range:mul( 255/(my_range[2]-my_range[1]) )
    
  -- Ensure that we are between 0 and 255
  adjusted_range[torch.lt(adjusted_range,0)] = 0
  adjusted_range[torch.gt(adjusted_range,255)] = 255
  type.mesh_byte:copy( adjusted_range )
  
  -- Compression
  local c_mesh
  if net_settings[2]==1 then
    -- jpeg
    metadata.c = 'jpeg'
    c_mesh = jpeg.compress_gray(
    type.mesh_byte:storage():pointer(),
    type.mesh_byte:size(2),
    type.mesh_byte:size(1) )
  elseif net_settings[2]==2 then
    -- png
    c_mesh = png.compress(
    type.mesh_byte:storage():pointer(),
    type.mesh_byte:size(2),
    type.mesh_byte:size(1),
    1 )
  end
  
  -- Perform the sending
  local meta = mp.pack(metadata)
  --mesh_pub_ch:send( {meta, payload} )
  local ret, err = mesh_udp_ch:send( meta..c_mesh )
  print(err or string.format('Sent a %g kB %s packet!', ret/1024, type.meta.name))
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
    -- Copy lidar readings to the torch object for fast modification
    ranges:tensor( chest.mesh:select(1,col), chest.res[2], chest.offset )
    -- Save the pan angle
    chest.lidarangles[col] = Body.get_lidar_position()[1]
  end
end

local function head_callback()
  local meta, has_more = head.lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_head_lidar()
  -- Insert into the correct column
  local row = tilt_to_row( metadata.hangle[2] )
  -- Only if a valid column is returned
  if row then
    -- Copy lidar readings to the torch object for fast modification
    ranges:tensor( head.mesh:select(1,row), head.res[2], head.offset )
    -- Save the pan angle
    head.lidarangles[row] = Body.get_head_position()[2]
  end
end

------------------------------
-- Polling with zeromq
local wait_channels = {}
if head.lidar_ch then
  head.lidar_ch.callback = head_callback
  table.insert( wait_channels, head.lidar_ch )
end
if chest.lidar_ch then
  chest.lidar_ch.callback = chest_callback
  table.insert( wait_channels, chest.lidar_ch )
end
local channel_polls = simple_ipc.wait_on_channels( wait_channels )

-- Begin polling
--channel_polls:start()
----[[
local channel_timeout = 100 --milliseconds
while true do
  local npoll = channel_polls:poll(channel_timeout)
  --if npoll<1 then print'timeout' end
  -- Stream the current mesh
  stream_mesh(head)
  stream_mesh(chest)
end
--]]