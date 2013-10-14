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
local function setup_mesh( name, tbl )
  tbl = tbl or {}
  -- Save the meta data for easy sending
  tbl.meta = {}
  -- Actuator endpoints
  -- In radians, specifies the actuator scanline angle endpoints
  -- The third number is the scanline density (scanlines/radian)
  local get_name = 'get_'..name
  tbl.meta.scanlines = vcm[get_name..'_scanlines']()
  -- Field of view endpoints of the lidar ranges
  -- -135 to 135 degrees for Hokuyo
  -- This is in RADIANS, though
  tbl.meta.fov = vcm[get_name..'_fov']()
  -- Depths when compressing
  tbl.meta.depths = vcm[get_name..'_depths']()
  tbl.meta.name = name
  -- Type of compression
  tbl.meta.c = 'jpeg'
  -- Timestamp
  tbl.meta.t = Body.get_time()
  tbl.meta.rpy = {0,0,0}

  -- Find the resolutions
  local scan_resolution = tbl.meta.scanlines[3]
    * math.abs(tbl.meta.scanlines[2]-tbl.meta.scanlines[1])
  scan_resolution = math.ceil(scan_resolution)

  local lidar_sensor_fov = vcm[get_name..'_sensor_fov']()
  local lidar_sensor_width = vcm[get_name..'_sensor_width']()
  
--  local reading_per_radian = 1 / (.25*math.pi/180)
  local reading_per_radian = 
    (lidar_sensor_width-1)/lidar_sensor_fov;

  tbl.meta.reading_per_radian = reading_per_radian
  
  local fov_resolution = reading_per_radian
    * math.abs(tbl.meta.fov[2]-tbl.meta.fov[1])
  fov_resolution = math.ceil(fov_resolution)

  -- Resolution
  tbl.meta.resolution = {scan_resolution,fov_resolution}
  print("Resolution:",scan_resolution,fov_resolution)
  -- TODO: Be able to resize these
  tbl.mesh_byte = torch.ByteTensor( scan_resolution, fov_resolution ):zero()
  tbl.mesh      = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  tbl.mesh_adj  = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  -- TODO: Save the exact actuator angles?
  tbl.scan_angles  = torch.DoubleTensor( scan_resolution ):zero()
  -- Find the offset for copying lidar readings into the mesh
  -- if fov is from -135 to 135 degrees, then offset_idx is zero
  -- if fov is from 0 to 135 degrees, then offset_idx is 540

  --We save robot pose for every frame
  --Otherwise the mesh will broken if the robot moves around  
  tbl.pose_upperbyte = torch.ByteTensor( scan_resolution, 3 ):zero()
  tbl.pose_lowerbyte = torch.ByteTensor( scan_resolution, 3 ):zero()

--  local fov_offset = 540+math.ceil( reading_per_radian*tbl.meta.fov[1] )
  local fov_offset = (lidar_sensor_width-1)/2+math.ceil( reading_per_radian*tbl.meta.fov[1] )
  tbl.offset_idx   = math.floor(fov_offset)
  --print('fov offset',name,fov_offset,tbl.offset_idx,fov_resolution)
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
  
  type.meta.depths = {near,far}

  local c_pose_upperbyte, c_pose_lowerbyte
  c_pose_upperbyte = zlib.compress(
    type.pose_upperbyte:storage():pointer(),
    type.pose_upperbyte:nElement() )

  c_pose_lowerbyte = zlib.compress(
    type.pose_lowerbyte:storage():pointer(),
    type.pose_lowerbyte:nElement() )

  type.meta.c_pose_upperbyte = c_pose_upperbyte
  type.meta.c_pose_lowerbyte = c_pose_lowerbyte

  return mp.pack(type.meta), c_mesh
end

-- type is head or chest table
local function stream_mesh(type)
  local get_name = 'get_'..type.meta.name
  -- Network streaming settings
  local net_settings = vcm[get_name..'_net']()
  -- Streaming

  if net_settings[1]==0 then return end
  -- Sensitivity range in meters
  -- Depths when compressing
  local depths = vcm[get_name..'_depths']()

  local metapack, c_mesh = prepare_mesh(
    type,
    depths[1],depths[2],
    net_settings[2])

  -- Sending to other processes
  --mesh_pub_ch:send( {meta, payload} )
  local ret, err = mesh_udp_ch:send( metapack..c_mesh)
  if err then print('mesh udp',err) end
  if net_settings[1]==1 then
    net_settings[1] = 0
    vcm['set_'..type.meta.name..'_net'](net_settings)
  end
  --[[
  print(err or string.format('Sent a %g kB packet.', ret/1024))
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
  local scanline = math.floor(ratio*res+.5)
  --local scanline = math.ceil(ratio*res)
  -- Return a bounded value
  return math.max( math.min(scanline, res), 1 )
end

--Convert a float number into two bytes
--We assume that the number is -127 to 128 
local function float_to_twobyte(num)
  local lowerbyte,upperbyte;
  num = (num*256)
  num = num + 32768;
  return math.floor(num/256), math.mod(num,256)
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
  if not scanline then return end -- Only if a valid column is returned

  --SJ: If lidar moves faster than the mesh scanline resolution, we need to fill the gap  
  --TODO: boundary lines may not be written at all, making glitches
  local scanlines = {}
  prev_dir =  vcm.get_chest_lidar_last_scan_dir()
  
  

  if chest.last_scanline then
    if scanline>chest.last_scanline then
      new_dir = 1
      if prev_dir==1 then
        for i=chest.last_scanline+1,scanline do table.insert(scanlines,i) end
      else
        --Fill the boundary
        print("Flip: 1 to ",chest.last_scanline,scanline)
        for i = 1, math.max(chest.last_scanline-1, scanline) do table.insert(scanlines,i) end        
      end
    elseif scanline<chest.last_scanline then
      new_dir = -1
      if prev_dir==-1 then
        for i=scanline,chest.last_scanline-1 do table.insert(scanlines,i)  end
      else
        --Fill the boundary
        print("Flip:",chest.last_scanline,scanline,"to",chest.meta.resolution[1])        
        for i = math.min(chest.last_scanline+1, scanline), chest.meta.resolution[1] do
          table.insert(scanlines,i) 
        end
        
      end
    else return end --Duplicate scanline: don't update
  else
    table.insert(scanlines,scanline) --First scanline. Just add one line
  end
  chest.last_scanline = scanline  

  --print("prev:",prev_dir, "new:",new_dir, chest.last_scanline)
  vcm.set_chest_lidar_last_scan_dir(new_dir)


  local pose = wcm.get_robot_pose()
  for i,line in ipairs(scanlines) do
      ranges:tensor( -- Copy lidar readings to the torch object for fast modification
        chest.mesh:select(1,line),
        chest.mesh:size(2),
        chest.offset_idx )      
      chest.scan_angles[line] = angle -- Save the pan angle

      chest.pose_upperbyte[line][1],
        chest.pose_lowerbyte[line][1]= float_to_twobyte(pose[1])
      chest.pose_upperbyte[line][2],
        chest.pose_lowerbyte[line][2]= float_to_twobyte(pose[2])
      chest.pose_upperbyte[line][3],
        chest.pose_lowerbyte[line][3]= float_to_twobyte(pose[3])       

  end  

  chest.meta.rpy = metadata.rpy --Save the body tilt info
  -- We've been updated
  chest.meta.t = metadata.t  
end

local function head_callback()
  local meta, has_more = head.lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Get raw data from shared memory
  local ranges = Body.get_head_lidar()
  -- Insert into the correct scanlin
  local angle = metadata.hangle[2]
  local scanline = angle_to_scanline( head.meta, angle )
  if not scanline then return end -- Only if a valid column is returned

  --SJ: If lidar moves faster than the mesh scanline resolution, we need to fill the gap  
  local scanlines = {}
  if head.last_scanline then
    if scanline>head.last_scanline then
      for i=head.last_scanline+1,scanline do 
        table.insert(scanlines,i)
      end
    elseif scanline<head.last_scanline then
      for i=scanline,head.last_scanline-1 do 
        table.insert(scanlines,i)
      end
    else return end --Duplicate scanline: don't update
  else
    table.insert(scanlines,scanline) --First scanline. Just add one line
  end
  head.last_scanline = scanline

  local pose = wcm.get_robot_pose()
  for i,line in ipairs(scanlines) do
      ranges:tensor( -- Copy lidar readings to the torch object for fast modification
        head.mesh:select(1,line),
        head.mesh:size(2),
        head.offset_idx )      
      head.scan_angles[line] = angle -- Save the pan angle

      head.pose_upperbyte[line][1],
        head.pose_lowerbyte[line][1]= float_to_twobyte(pose[1])
      head.pose_upperbyte[line][2],
        head.pose_lowerbyte[line][2]= float_to_twobyte(pose[2])
      head.pose_upperbyte[line][3],
        head.pose_lowerbyte[line][3]= float_to_twobyte(pose[3])       
  end
  head.meta.rpy = metadata.rpy --Save the body tilt info
  -- We've been updated
  head.meta.t = metadata.t
end

local function reliable_callback()
  local request, has_more
  repeat
    request, has_more = mesh_tcp_ch:receive()
    request = mp.unpack(request)
    util.ptable(request)
    local metapack, c_mesh = prepare_mesh(
      mesh_lookup[request.type],
      request.near or 0,request.far or 5,
      request.c or 2)
    -- NOTE: The zmq channel is REP/REQ
    -- Reply with the result of the request
    local ret = mesh_tcp_ch:send( {metapack, c_mesh} )
    print('ret rel',ret)
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
  chest = setup_mesh'chest_lidar'
  head  = setup_mesh'head_lidar'

  -- Poll the lidar readings with zeromq
  local wait_channels = {}
  if head then
    mesh_lookup['head_lidar'] = head
    -- Subscribe to a lidar channel
    local ch = simple_ipc.new_subscriber('head_lidar')
    ch.callback = head_callback
    table.insert( wait_channels, ch )
    head.lidar_ch  = ch
  end
  if chest then
    mesh_lookup['chest_lidar'] = chest
    -- Subscribe to a lidar channel
    local ch = simple_ipc.new_subscriber'chest_lidar'
    ch.callback = chest_callback
    table.insert( wait_channels, ch )
    chest.lidar_ch  = ch
  end

  -- Reliable request/reply
  --mesh_tcp_ch = simple_ipc.new_replier(Config.net.reliable_mesh,'*')
  if mesh_tcp_ch then
    mesh_tcp_ch.callback = reliable_callback
    table.insert( wait_channels, mesh_tcp_ch )
  end

  -- Send mesh messages on interprocess to other processes
  -- TODO: Not used yet
  mesh_pub_ch = simple_ipc.new_publisher'mesh'

  -- Send (unreliably) to users
  mesh_udp_ch = udp.new_sender(
    Config.net.operator.wired, Config.net.mesh )
  print('Connected to Operator:',
    Config.net.operator.wired,Config.net.mesh)

  -- Prepare the polling
  channel_polls = simple_ipc.wait_on_channels( wait_channels )
end

function mesh.update()
  local npoll = channel_polls:poll(channel_timeout)
  -- Stream the current mesh  
  stream_mesh(head)
  stream_mesh(chest)
  -- Check if we must update our torch data
  if head.meta.scanlines ~= vcm.get_head_lidar_scanlines() or
    head.meta.fov ~= vcm.get_head_lidar_fov()
    then
    setup_mesh('head_lidar',head)
  end
  if chest.meta.scanlines ~= vcm.get_chest_lidar_scanlines() or
    chest.meta.fov ~= vcm.get_chest_lidar_fov()
    then
    setup_mesh('chest_lidar',chest)
  end  
end

function mesh.exit()
end

mesh.entry()
while true do mesh.update() end
mesh.exit()

return mesh
