-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'../include.lua'

-- Libraries
local torch      = require'torch'
torch.Tensor     = torch.DoubleTensor
local Body       = require'Body'
local util       = require'util'
local jpeg       = require'jpeg'
local zlib       = require'zlib'
local util       = require'util'
local mp         = require'msgpack'
local carray     = require'carray'
local simple_ipc = require'simple_ipc'
local udp        = require'udp'
require'vcm'

-- Globals
-- Output channels
local mesh_pub_ch, mesh_udp_ch, mesh_tcp_ch
-- Input channels
local channel_polls
local channel_timeout = 100 --milliseconds
-- Data structures
local chest, head
local mesh_lookup = {}

jpeg.set_quality( 95 )

-- Setup metadata and tensors for a lidar mesh
local function setup_mesh( name )
  local tbl = {}
  -- Save the meta data for easy sending
  tbl.meta = {}
  -- Actuator endpoints
  -- In radians, specifices the actuator scanline angle endpoints
  -- The third number is the scanline density (scanlines/radian)
  tbl.meta.scanlines = vcm['get_'..name..'_lidar_scanlines']()
  -- Field of view endpoints of the lidar ranges
  -- -135 to 135 degrees for Hokuyo
  -- This is in RADIANS, though
  tbl.meta.fov = vcm['get_'..name..'_lidar_fov']()
  -- Depths when compressing
  tbl.meta.depths = vcm['get_'..name..'_lidar_depths']()
  tbl.meta.name = name
  -- Type of compression
  tbl.meta.c = 'jpeg'
  -- Timestamp
  tbl.meta.t = Body.get_time()

  -- Find the resolutions
  local scan_resolution = tbl.meta.scanlines[3]
    * math.abs(tbl.meta.scanlines[2]-tbl.meta.scanlines[1])
  scan_resolution = math.ceil(scan_resolution)
  local reading_per_radian = 1 / (.25*math.pi/180)
  local fov_resolution = reading_per_radian
    * math.abs(tbl.meta.fov[2]-tbl.meta.fov[1])
  fov_resolution = math.ceil(fov_resolution)

  -- Resolution
  tbl.meta.resolution = {scan_resolution,fov_resolution}

  -- TODO: Be able to resize these
  tbl.mesh_byte = torch.ByteTensor( scan_resolution, fov_resolution ):zero()
  tbl.mesh      = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  tbl.mesh_adj  = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  -- TODO: Save the exact actuator angles?
  tbl.scan_angles  = torch.DoubleTensor( scan_resolution ):zero()
  -- Subscribe to a lidar channel
  tbl.lidar_ch  = simple_ipc.new_subscriber(name..'_lidar')
  -- Find the offset for copying lidar readings into the mesh
  -- if fov is from -135 to 135 degrees, then offset_idx is zero
  -- if fov is from 0 to 135 degrees, then offset_idx is 540
  tbl.offset_idx = math.floor((1081-fov_resolution)/2)
  -- For streaming
  tbl.needs_update = true
  return tbl
end

local function prepare_mesh(type,near,far,method)
  -- Safety check
  if near>=far then return end
  if not type then return end

  -- Enhance the dynamic range of the mesh image
  local adjusted_range = type.mesh_adj
  adjusted_range:copy(type.mesh):add( -near )
  adjusted_range:mul( 255/(far-near) )
    
  -- Ensure that we are between 0 and 255
  adjusted_range[torch.lt(adjusted_range,0)] = 0
  adjusted_range[torch.gt(adjusted_range,255)] = 255
  type.mesh_byte:copy( adjusted_range )
  
  -- Compression
  local c_mesh
  local dim = type.mesh_byte:size()
  if method==1 then
    -- jpeg
    type.meta.c = 'jpeg'
    c_mesh = jpeg.compress_gray( type.mesh_byte:storage():pointer(),
      dim[2], dim[1] )
  elseif method==2 then
    -- zlib
    type.meta.c = 'zlib'
    c_mesh = zlib.compress(
      type.mesh_byte:storage():pointer(),
      type.mesh_byte:nElement() )
  elseif method==3 then
    -- png
    type.meta.c = 'png'
    c_mesh = png.compress(type.mesh_byte:storage():pointer(),
      dim[2], dim[1], 1)
  else
    -- raw data?
    return
  end
  return mp.pack(type.meta), c_mesh
end

-- type is head or chest table
local function stream_mesh(type)
  -- Network streaming settings
  local net_settings = vcm['get_'..type.meta.name..'_lidar_net']()
  -- Streaming
  if net_settings[1]==0 then return end
  -- Sensitivity range in meters
  -- Depths when compressing
  local depths = vcm['get_'..type.meta.name..'_lidar_depths']()
  type.meta.depths = depths
  local near = depths[1]
  local far = depths[2]

  local metapack, c_mesh = prepare_mesh(
    type,depths[1],depths[2],net_settings[2])

  --mesh_pub_ch:send( {meta, payload} )
  local ret, err = mesh_udp_ch:send( metapack..c_mesh )
  if net_settings[1]==1 then
    net_settings[1] = 0
    vcm['set_'..type.meta.name..'_lidar_net'](net_settings)
  end
  --[[
  print(err or string.format('Sent a %d by %d %g kB %s packet from %g to %g.', 
      dim[2], dim[1], ret/1024, type.meta.name, near, far))
  --]]
end

------------------------------
-- Data copying helpers
-- Convert a pan angle to a column of the chest mesh image
local function angle_to_scanline( meta, rad )
  local start = meta.scanlines[1]
  local stop  = meta.scanlines[2]
  local res   = meta.resolution[1]
  local ratio = (rad-start)/(stop-start)
  -- Round
  --local scanline = math.floor(ratio*res+.5)
  local scanline = math.ceil(ratio*res)
  -- Return a bounded value
  return math.max( math.min(scanline, res), 1 )
end

------------------------------
-- Lidar Callback functions --
------------------------------
local function chest_callback()
  local meta, has_more = chest.lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_chest_lidar()
  -- Insert into the correct column
  local angle = metadata.pangle
  local scanline = angle_to_scanline( chest.meta, angle )
  -- Only if a valid column is returned
  if scanline then
    -- Copy lidar readings to the torch object for fast modification
    ranges:tensor(
      chest.mesh:select(1,scanline),
      chest.mesh:size(2),
      chest.offset_idx )
    -- Save the pan angle
    chest.scan_angles[scanline] = angle
    -- We've been updated
    chest.needs_update = true
    chest.meta.t     = metadata.t
  end
end

local function head_callback()
  local meta, has_more = head.lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_head_lidar()
  -- Insert into the correct scanlin
  local angle = metadata.hangle[2]
  local scanline = angle_to_scanline( head.meta, angle )
  -- Only if a valid column is returned
  if scanline then
    -- Copy lidar readings to the torch object for fast modification
    ranges:tensor(
      head.mesh:select(1,scanline), -- scanline
      head.mesh:size(2),
      head.offset_idx )
    -- Save the pan angle
    head.scan_angles[scanline] = angle
    head.needs_update = true
    head.meta.t = metadata.t
  end
end

local function reliable_callback()
  local request, has_more
  repeat
    request, has_more = mesh_tcp_ch:receive()
    request = mp.unpack(request)
    --util.ptable(request)
    local metapack, c_mesh = prepare_mesh(
      mesh_lookup[request.type],
      request.near,request.far,
      request.c)
    -- NOTE: The zmq channel is REP/REQ
    -- Reply with the result of the request
    local ret = mesh_tcp_ch:send( {metapack, c_mesh} )
  until not has_more
end

------------------
-- Main routine --
------------------

-- Make an object
local mesh = {}
-- Entry function
function mesh.entry()
  -- Setup the data structures for each mesh
  chest = setup_mesh'chest'
  mesh_lookup['chest'] = chest
  head  = setup_mesh'head'
  mesh_lookup['head'] = head

  -- Poll the lidar readings with zeromq
  local wait_channels = {}
  if head.lidar_ch then
    head.lidar_ch.callback = head_callback
    table.insert( wait_channels, head.lidar_ch )
  end
  if chest.lidar_ch then
    chest.lidar_ch.callback = chest_callback
    table.insert( wait_channels, chest.lidar_ch )
  end

  -- Reliable request/reply
  mesh_tcp_ch = simple_ipc.new_replier(
    Config.net.reliable_mesh,'*')
  if mesh_tcp_ch then
    print('add reliable')
    mesh_tcp_ch.callback = reliable_callback
    table.insert( wait_channels, mesh_tcp_ch )
  end

  -- Send mesh messages on interprocess to other processes
  -- TODO: Not used yet
  mesh_pub_ch = simple_ipc.new_publisher'mesh'

  -- Send (unreliably) to users
  mesh_udp_ch = udp.new_sender(
    Config.net.operator.wireless, Config.net.mesh )

  -- Prepare the polling
  channel_polls = simple_ipc.wait_on_channels( wait_channels )
end

function mesh.update()
  local npoll = channel_polls:poll(channel_timeout)
  --print('here',npoll,head.needs_update,chest.needs_update)
  -- Stream the current mesh
  stream_mesh(head)
  stream_mesh(chest)
end

function mesh.exit()
end

mesh.entry()
while true do mesh.update() end
mesh.exit()

return mesh
