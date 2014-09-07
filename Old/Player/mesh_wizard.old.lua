-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------

dofile'../include.lua'

-- Libraries
local torch      = require'torch'
torch.Tensor     = torch.DoubleTensor
local Body       = require'Body'
local util       = require'util'
local jpeg       = require'jpeg'
local png        = require'png'
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

  -- Grab the sensor paramters
  local lidar_sensor_fov, lidar_sensor_width = 
    unpack(vcm[get_name..'_sensor_params']())
    
  print('Mesh |',name,lidar_sensor_fov, lidar_sensor_width)
  
  -- Set our resolution
  local reading_per_radian = 
    (lidar_sensor_width-1)/lidar_sensor_fov;

  tbl.meta.reading_per_radian = reading_per_radian
  
  local fov_resolution = reading_per_radian
    * math.abs(tbl.meta.fov[2]-tbl.meta.fov[1])
  fov_resolution = math.ceil(fov_resolution)

  -- Resolution
  tbl.meta.resolution = {scan_resolution,fov_resolution}
  
  -- Pose information
--  tbl.meta.pose = {}
--  for i=1,scan_resolution do tbl.meta.pose[i] = {0,0,0} end
  tbl.meta.posex = {}
  tbl.meta.posey = {}
  tbl.meta.posez = {}
  for i=1,scan_resolution do 
    tbl.meta.posex[i],
    tbl.meta.posey[i],
    tbl.meta.posez[i]=0,0,0
  end

  -- In memory mesh
  tbl.mesh      = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  -- Mesh buffers for compressing and sending to the user
  tbl.mesh_byte = torch.ByteTensor( scan_resolution, fov_resolution ):zero()
  tbl.mesh_adj  = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  -- Save the exact actuator angles of every scan
  tbl.scan_angles  = torch.DoubleTensor( scan_resolution ):zero()
  -- Save the exact pose of every scan
  -- TODO: Save the whole whole rotation?
  --tbl.scan_poses = torch.FloatTensor( scan_resolution, 3 ):zero()
  -- metadata to point here
  --tbl.meta.poses = torch.FloatTensor( tbl.scan_poses:storage() )

  -- Find the offset for copying lidar readings into the mesh
  -- if fov is from -fov/2 to fov/2 degrees, then offset_idx is zero
  -- if fov is from 0 to fov/2 degrees, then offset_idx is sensor_width/2
  local fov_offset = (lidar_sensor_width-1)/2+math.ceil( reading_per_radian*tbl.meta.fov[1] )
  tbl.offset_idx   = math.floor(fov_offset)
  return tbl
end

local function prepare_mesh(mesh,depths,net_settings)
  -- Safety check
  local near, far = unpack(depths)
  if near>=far then
		print('near ',near,' far ',far,'mixed')
		return
	end
  if not mesh then
		print('nil mesh')
		return
	end

  local stream, method, quality, use_pose = unpack(net_settings)

  -- Enhance the dynamic range of the mesh image
  local adjusted_range = mesh.mesh_adj
  adjusted_range:copy(mesh.mesh):add( -near )
  adjusted_range:mul( 255/(far-near) )
    
  -- Ensure that we are between 0 and 255
	local mesh_byte = mesh.mesh_byte
  adjusted_range[torch.lt(adjusted_range,0)] = 0
  adjusted_range[torch.gt(adjusted_range,255)] = 255
  mesh_byte:copy( adjusted_range )
  
  -- Compression
  local c_mesh 
  local dim = mesh_byte:size()
  if method==1 then
    -- jpeg
    mesh.meta.c = 'jpeg'
    jpeg.set_quality( quality )
    c_mesh = jpeg.compress_gray( mesh_byte:storage():pointer(),
      dim[2], dim[1] )
  elseif method==2 then
    -- zlib
    mesh.meta.c = 'zlib'
    c_mesh = zlib.compress(
      mesh_byte:storage():pointer(),
      mesh_byte:nElement() )
  elseif method==3 then
    -- png
    mesh.meta.c = 'png'
    c_mesh = png.compress(mesh_byte:storage():pointer(),
      dim[2], dim[1], 1)
  else
    -- raw data?
    return
  end
  -- Depth data is compressed to a certain range
  mesh.meta.depths = {near,far}

  return mp.pack(mesh.meta), c_mesh
end

-- mesh is head or chest table
local function stream_mesh(mesh)
  local get_name = 'get_'..mesh.meta.name
  -- Network streaming settings
  local net_settings = vcm[get_name..'_net']()
  -- Streaming?
  if net_settings[1]==0 then return end

  -- Check if the panning is over (for interval streaming)
  if net_settings[1]==2 or net_settings[1]==4 then
    if mesh.current_direction == mesh.prev_direction then return end
  end

  -- Sensitivity range in meters
  -- Depths when compressing
  local depths = vcm[get_name..'_depths']()

  local metapack, c_mesh = prepare_mesh(
    mesh,
    depths,
    net_settings)
  
	if net_settings[1]==1 then
    -- Unreliable Single sending
    net_settings[1] = 0
    local ret, err = mesh_udp_ch:send( metapack..c_mesh )
    if err then print('mesh udp',err) end
    vcm['set_'..mesh.meta.name..'_net'](net_settings)
		print('Mesh | sent single unreliable!',mesh.meta.t)
  elseif net_settings[1]==2 then
    -- Unreliable Interval streaming
    local ret, err = mesh_udp_ch:send( metapack..c_mesh )
    if err then print('mesh udp',err) end
    vcm['set_'..mesh.meta.name..'_net'](net_settings)
    print('Mesh | sent interval unreliable!',mesh.meta.t)
  elseif net_settings[1]==3 then
    -- Reliable single frame
    net_settings[1] = 0
    local ret = mesh_tcp_ch:send{metapack,c_mesh}
    vcm['set_'..mesh.meta.name..'_net'](net_settings)
		print('Mesh | sent single reliable!',mesh.meta.t)
  elseif net_settings[1]==4 then
    -- Reliable Interval streaming
    local ret = mesh_tcp_ch:send{metapack,c_mesh}
    vcm['set_'..mesh.meta.name..'_net'](net_settings)
		print('Mesh | sent interval reliable!',mesh.meta.t)
  end
  --[[
  util.ptable(mesh.meta)
  print(err or string.format('Sent a %g kB packet.', ret/1024))
  --]]
end

------------------------------
-- Data copying helpers
-- Convert a pan angle to a column of the chest mesh image
local function angle_to_scanlines( lidar, rad )
  -- Get the most recent direction the lidar was moving
  local prev_scanline = lidar.current_scanline
  -- Get the metadata for calculations
  local meta  = lidar.meta
  local start = meta.scanlines[1]
  local stop  = meta.scanlines[2]
  local res   = meta.resolution[1]
  local ratio = (rad-start)/(stop-start)
  -- Round
  local scanline = math.floor(ratio*res+.5)

  -- Return a bounded value
  scanline = math.max( math.min(scanline, res), 1 )

  --SJ: I have no idea why, but this fixes the scanline tilting problem
  if lidar.current_direction then
    if lidar.current_direction<0 then        
      scanline = math.max(1,scanline-1)
    else
      scanline = math.min(res,scanline+1)
    end
  end

  -- Save in our table
  lidar.current_scanline = scanline
	-- Initialize if no previous scanline
  -- If not moving, assume we are staying in the previous direction
	if not prev_scanline then return {scanline} end
  -- Grab the most recent scanline saved in the mesh
  local prev_direction = lidar.current_direction
  if not prev_direction then
    lidar.current_direction = 1
    return {scanline}
  end
  -- Grab the direction
  local direction
  local diff_scanline = scanline - prev_scanline
  if diff_scanline==0 then
    direction = prev_direction
  else
    direction = util.sign(diff_scanline)
  end
  -- Save the directions
  lidar.current_direction = direction
  lidar.prev_direction = prev_direction

  -- Find the set of scanlines for copying the lidar reading
  local scanlines = {}
  if direction==prev_direction then
    -- fill all lines between previous and now
    for s=prev_scanline+1,scanline,direction do table.insert(scanlines,s) end
  else
    -- Changed directions!
    -- Populate the borders, too
    if direction>0 then
      -- going away from 1 to end
      local start_line = math.min(prev_scanline+1,scanline)
      for s=start_line,res do table.insert(scanlines,i) end
    else
      -- going away from end to 1
      local end_line = math.max(prev_scanline-1,scanline)
      for s=1,end_line do table.insert(scanlines,i) end        
    end
  end

  -- Return for populating
  return scanlines
end

------------------------------
-- Lidar Callback functions --
------------------------------
local function chest_callback()
  local meta, has_more = chest.lidar_ch:receive()
  local metadata = mp.unpack(meta)
  
  if chest.meta.scanlines ~= vcm.get_chest_lidar_scanlines() or
    chest.meta.fov ~= vcm.get_chest_lidar_fov()
    then
    setup_mesh('chest_lidar',chest)
    chest.current_scanline = nil --We need to clear this
    print("Chest Resolution:",unpack(chest.meta.resolution))
  end
  
  -- Get raw data from shared memory
  local ranges = Body.get_chest_lidar()
  -- Save the body pose info
  local pose = metadata.pose
  -- Insert into the correct column
  local angle = metadata.pangle
  local scanlines = angle_to_scanlines( chest, angle )
  -- Update each outdated scanline in the mesh
  for _,line in ipairs(scanlines) do
		-- Copy lidar readings to the torch object for fast modification
    ranges:tensor( 
			chest.mesh:select(1,line),
      chest.mesh:size(2),
      chest.offset_idx )
		-- Save the pan angle
		chest.scan_angles[line] = angle
    -- Save the pose
    --[[
    local chest_pose = chest.scan_poses:select(1,line)
    chest_pose[1],chest_pose[2],chest_pose[3] = unpack(pose)
    --]]
--    chest.meta.pose[line] = {unpack(pose)}

    chest.meta.posex[line],
    chest.meta.posey[line],
    chest.meta.posez[line]=
    pose[1],pose[2],pose[3]

  end
  -- Save the body tilt info
  -- TODO: bodyHeight as well?
  chest.meta.rpy = metadata.rpy
  -- We've been updated at this timestamp
  chest.meta.t = metadata.t
end

local function head_callback()
  local meta, has_more = head.lidar_ch:receive()
  local metadata = mp.unpack(meta)
  
  -- Check if we must update our torch data
  if head.meta.scanlines ~= vcm.get_head_lidar_scanlines() or
    head.meta.fov ~= vcm.get_head_lidar_fov()
    then
    setup_mesh('head_lidar',head)
    print("Head Resolution:",unpack(head.meta.resolution))
  end
  
  
  -- Get raw data from shared memory
  local ranges = Body.get_head_lidar()
  -- Save the body pose info
  local pose = metadata.pose
  -- Insert into the correct scanlin
  local angle = metadata.hangle[2]
  local scanlines = angle_to_scanlines( head, angle )
  -- Update each outdated scanline in the mesh
  for _,line in ipairs(scanlines) do
		-- Copy lidar readings to the torch object for fast modification
    ranges:tensor(
			head.mesh:select(1,line),
      head.mesh:size(2),
      head.offset_idx )      
    -- Save the pan angle
    head.scan_angles[line] = angle
    -- Save the pose
    local head_pose = head.scan_poses:select(1,line)
    head_pose[1],head_pose[2],head_pose[3] = unpack(pose)      
  end
  -- Save the body tilt info
  -- TODO: bodyHeight as well?
  head.meta.rpy = metadata.rpy
  -- We've been updated
  head.meta.t = metadata.t
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

  -- Reliable tcp sending
  mesh_tcp_ch = simple_ipc.new_publisher(
    Config.net.reliable_mesh,false,'*') --Config.net.operator.wired

  -- Send mesh messages on interprocess to other processes
  -- TODO: Not used yet
  --mesh_pub_ch = simple_ipc.new_publisher'mesh'

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
end

function mesh.exit()
end
mesh.entry()
while true do mesh.update() end
mesh.exit()

return mesh
