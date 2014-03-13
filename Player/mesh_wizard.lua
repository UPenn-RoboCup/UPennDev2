-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------
dofile'../include.lua'
-- Going to be threading this
local simple_ipc = require'simple_ipc'
-- Are we a child?
local IS_CHILD, pair_ch = false, nil
local CTX, metadata = ...
if CTX and type(metadata)=='table' then
	IS_CHILD=true
	simple_ipc.import_context( CTX )
	pair_ch = simple_ipc.new_pair(metadata.ch_name)
	print('CHILD CTX')
end

local lidars = {'lidar0','lidar1'}

-- The main thread's job is to give:
-- joint angles and network request
if not IS_CHILD then
	-- Only the parent communicates with the body
	local Body = require'Body'
	-- TODO: Signal of ctrl-c should kill all threads...
	--local signal = require'signal'
	-- Only the parent accesses shared memory
	require'vcm'
	local poller, threads, channels = nil, {}, {}
	-- Spawn children
	for i,v in ipairs(lidars) do
		local meta = {
			name = v,
			fov = {-60*DEG_TO_RAD,60*DEG_TO_RAD},
			scanlines = {-45*DEG_TO_RAD,45*DEG_TO_RAD},
			density = 10*RAD_TO_DEG, -- #scanlines per radian on actuated motor
			dynrange = {.1,1}, -- Dynamic range of depths when compressing
			c = 'jpeg', -- Type of compression
			t = Body.get_time(),
		}
		local thread, ch = simple_ipc.new_thread('mesh_wizard.lua',v,meta)
		ch.callback = function(s)
			local data, has_more = poller.lut[s]:receive()
			print('child tread data!',data)
		end
		threads[v] = {thread,ch}
		table.insert(channels,ch)
		-- Start officially
		thread:start()
	end
	-- TODO: Also poll on mesh requests, instead of using net_settings...
	-- This would be a Replier (then people ask for data, or set data)
	-- Instead of changing shared memory a lot...
	local replier = simple_ipc.new_replier'mesh'
	replier.callback = function(s)
		local data, has_more = poller.lut[s]:receive()
		-- Send message to a thread
	end
	table.insert(channels,replier)
	-- Just wait for requests from the children
	poller = simple_ipc.wait_on_channels(channels)
	--poller:start()
	local t_check = 0
	while poller.n>0 do
		local npoll = poller:poll(1e3)
		-- Check if everybody is alive periodically
		print(npoll,poller.n)
		local t = Body.get_time()
		if t-t_check>1e3 or npoll<1 then
			t_check = t
			for k,v in pairs(threads) do
				local thread, ch = unpack(v)
				if not thread:alive() then
					thread:join()
					poller:clean(ch.socket)
					threads[k] = nil
					print('Mesh |',k,'thread died!')
				else
					ch:send'hello'
				end
			end
		end -- if too long
	end
	return
end

-- Libraries
local torch      = require'torch'
torch.Tensor     = torch.DoubleTensor
local util       = require'util'
local jpeg       = require'jpeg'
jpeg.set_quality( 95 )
local png        = require'png'
local zlib       = require'zlib'
local util       = require'util'
local mp         = require'msgpack'
local carray     = require'carray'
local udp        = require'udp'

-- Globals
-- Output channels
local mesh_pub_ch, mesh_udp_ch, mesh_tcp_ch
-- Input channels
local channel_polls
local channel_timeout = 100 --milliseconds

-- Setup metadata and tensors for a lidar mesh
local reading_per_radian, scan_resolution, fov_resolution
local mesh, mesh_byte, mesh_adj, scan_angles
local offset_idx
local function setup_mesh( meta )
  -- Find the resolutions
  local scan_resolution = meta.density
    * math.abs(meta.scanlines[2]-meta.scanlines[1])
  scan_resolution = math.ceil(scan_resolution)
  -- Set our resolution
	-- NOTE: This has been changed in the lidar msgs...
  reading_per_radian = (1081-1)/(270*DEG_TO_RAD);
  fov_resolution = reading_per_radian * math.abs(meta.fov[2]-meta.fov[1])
  fov_resolution = math.ceil(fov_resolution)
  -- TODO: Pose information
	--[[
  tbl.meta.posex = {}
  tbl.meta.posey = {}
  tbl.meta.posez = {}
  for i=1,scan_resolution do 
    tbl.meta.posex[i],
    tbl.meta.posey[i],
    tbl.meta.posez[i]=0,0,0
  end
	--]]
  -- In-memory mesh
  mesh      = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  -- Mesh buffers for compressing and sending to the user
  mesh_byte = torch.ByteTensor( scan_resolution, fov_resolution ):zero()
  mesh_adj  = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  -- Save the exact actuator angles of every scan
  scan_angles  = torch.DoubleTensor( scan_resolution ):zero()
  -- Find the offset for copying lidar readings into the mesh
  -- if fov is from -fov/2 to fov/2 degrees, then offset_idx is zero
  -- if fov is from 0 to fov/2 degrees, then offset_idx is sensor_width/2
  local fov_offset = (1081-1)/2+math.ceil( reading_per_radian*meta.fov[1] )
  offset_idx   = math.floor(fov_offset)
end
-- Initial setup of the mesh from metadata
setup_mesh(metadata)
print('SETUP THE MESH!')

-- Poll for the lidar and master thread info
local lidar_ch = simple_ipc.new_subscriber(metadata.name)
lidar_ch.callback = function(s)
	local data, has_more = poller.lut[s]:receive()
	-- Send message to a thread
	print('Got lidar data!')
end
pair_ch.callback = function(s)
	local ch = poller.lut[s]
	local data, has_more = ch:receive()
	print('Got pair data!',data)
	-- Send message to a thread
	ch:send('what??')
end

local wait_channels = {}
table.insert(wait_channels,lidar_ch)
table.insert(wait_channels,pair_ch)
poller = simple_ipc.wait_on_channels( wait_channels )
print('start child poll')
poller:start()






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
