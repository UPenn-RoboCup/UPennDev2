-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'../include.lua'
--local use_udp = true

-- Libraries
require 'unix'
local torch      = require'torch'
torch.Tensor     = torch.DoubleTensor
local simple_ipc = require'simple_ipc'
local udp        = require'udp'
local carray     = require'carray'
local util       = require'util'
local jpeg       = require'jpeg'
-- Use high quality jpeg as the default.  TODO: Grab from vcm
jpeg.set_quality( 90 )
local mp         = require'msgpack'
local Body       = require'Body'
local util       = require'util'
local udp_port   = 43288
local udp_target = '192.168.123.23'

-- Sending the mesh replies on zmq or UDP
local mesh_rep_zmq = simple_ipc.new_publisher'mesh_response'
-- Receiving LIDAR ranges
local head_lidar_ch  = simple_ipc.new_subscriber'head_lidar'
local chest_lidar_ch = simple_ipc.new_subscriber'chest_lidar'
-- Sending the mesh requests on zmq or UDP
local mesh_req_zmq   = simple_ipc.new_subscriber'mesh_request'

if use_udp then
mesh_rep_udp = udp.new_sender( udp_target, udp_port )
mesh_req_udp   = udp.new_receiver( udp_port )
end


-- Robot-wide used Mesh images parameters
-- TODO: Contuously use these in LidarFSM
-- Desired panning resolution
local chest_res       = vcm.get_chest_lidar_mesh_resolution()
-- Panning endpoints
local chest_start     = vcm.get_chest_lidar_endpoints()[1]
local chest_stop      = vcm.get_chest_lidar_endpoints()[2]
-- TODO: Add some sort of "is_scanning check"

-- TODO: Get 1081 as a Body parameter for the size of the lidar
local chest_offset    = math.floor( (1081-chest_res[2])/2 )
-- TODO: Be able to resize these
local chest_mesh_byte = torch.ByteTensor( unpack(chest_res) ):zero()
local chest_mesh      = torch.FloatTensor( unpack(chest_res) ):zero()

-- Convert a pan angle to a column of the mesh image
local function pan_to_column( rad )
  rad = math.max( math.min(rad, chest_stop), chest_start )
  local ratio = (rad-chest_start)/(chest_stop-chest_start)
  local col = math.floor(ratio*chest_res[2]+.5)
  -- Do not return a column number if out of bounds
  if col>=1 and col<=chest_res[1] then return col end
end

local function reply_chest(chest_range)
  
  -- Safety check
  if chest_range[1]>=chest_range[2] then return end
  
  -- Grab the time of compression
  local t = unix.time()
  

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
  chest_mesh_byte:size(2),
  chest_mesh_byte:size(1) )
 
  -- TODO: Associate metadata with this depth image
  local meta_chest = {}
  meta_chest.t = t

  return meta_chest, jdepth
    
end

------------------------------
-- Lidar Callback functions
local function chest_callback()
  
  local meta, has_more = chest_lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_chest_lidar()
  -- Insert into the correct column
  local col = pan_to_column(metadata.pangle)
  -- Only if a valid column is returned
  if col then
    local chest_slice = chest_mesh:select(1,col)
    -- Copy to the torch object for fast modification
    ranges:tensor( chest_slice, chest_res[2], chest_offset )
  end
  
end

local function head_callback()
  local meta, has_more = head_lidar_ch:receive()
end

------------------------------
-- Request Callback functions
local function zmq_request_callback()
  local request, has_more = mesh_req_zmq:receive()
  local params = mp.unpack(request)
  if params.type=='chest' then
    jpeg.set_quality( params.quality or 90 )
    local metadata, payload = reply_chest(params.range or {0.10,5.00})
    if not metadata then return end
    local meta = mp.pack(metadata)
    mesh_rep_zmq:send( {meta, payload} )
    print('Sent a mesh!', #meta, #payload)
  end
  if params.type=='modify' then
    print("Modifying")
    util.ptable(params)
  end
end

local function udp_request_callback()
  local request = nil
  while mesh_req_udp:size()>0 do request = mesh_req_udp:receive() end
  local params = mp.unpack(request)
  if params.type=='chest' then
    jpeg.set_quality( params.quality or 90 )
    local metadata, payload = reply_chest(params.range or {0.10,5.00})
    if not metadata then return end
    metadata.sz = #payload
    local packet = mp.pack(metadata)..payload
    local ret, err = mesh_rep_udp:send( packet, #packet )
    if ret==-1 then
      print(err)
    else
      print( string.format('Sent a %d kB mesh packet!', ret / 1024) )
    end
  end
  if params.type=='modify' then
    print("Modifying")
    util.ptable(params)
  end
end
------------------------------
-- Polling with zeromq

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
if mesh_req_zmq then
  mesh_req_zmq.callback = zmq_request_callback
  table.insert( wait_channels, mesh_req_zmq )
end
if mesh_req_udp then
  print('Listening on',udp_port)
  local mesh_req_udp_poll = {}
  mesh_req_udp_poll.socket_handle = mesh_req_udp:descriptor()
  mesh_req_udp_poll.callback      = udp_request_callback
  table.insert( wait_channels, mesh_req_udp_poll )
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