-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------
-- TODO: Critical section for include
-- Something there is non-reentrant
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
-- For all threads
--local mp     = require'msgpack'
local mp     = require'msgpack.MessagePack'

local lidars = {'lidar0','lidar1'}

-- The main thread's job is to give:
-- joint angles and network request
if not IS_CHILD then
	-- TODO: Signal of ctrl-c should kill all threads...
	local signal = require'signal'
	-- Only the parent accesses shared memory
	require'vcm'
	local poller, channels = nil, {}
	-- Spawn children
	for i,v in ipairs(lidars) do
		local meta = {
			name = v,
			fov = {-60*DEG_TO_RAD,60*DEG_TO_RAD},
			scanlines = {-45*DEG_TO_RAD,45*DEG_TO_RAD},
			density = 10*RAD_TO_DEG, -- #scanlines per radian on actuated motor
			dynrange = {.1,1}, -- Dynamic range of depths when compressing
			c = 'jpeg', -- Type of compression
			t = 0,
		}
		-- The thread will automagically start detached upon gc
		local ch, thread = simple_ipc.new_thread('mesh_wizard.lua',v,meta)
		ch.callback = function(s)
			local data, has_more = poller.lut[s]:receive()
			--print('child tread data!',data)
		end
		-- Add the channel for our poller to sample
		channels[v] = ch
		-- Start detached
		thread:start(true,true)
	end
	-- TODO: Also poll on mesh requests, instead of using net_settings...
	-- This would be a Replier (then people ask for data, or set data)
	-- Instead of changing shared memory a lot...
	local replier = simple_ipc.new_replier'mesh'
	replier.callback = function(s)
		local data, has_more = poller.lut[s]:receive()
		-- Send message to a thread
		local cmd = mp.unpack(data)
		if cmd.id==0 then 
	end
	table.insert(channels,replier)
	-- Ensure that we shutdown the threads properly
	signal.signal("SIGINT", os.exit)
	signal.signal("SIGTERM", os.exit)
	-- Just wait for requests from the children
	poller = simple_ipc.wait_on_channels(channels)
	poller:start()
	return
end

-- Libraries
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor
-- NOTE: torch cannot use OpenBLAS in this thread
-- since OpenBLAS cannot operate in a threaded environmenta
-- TODO: Add a method to dynamically set N_THREADS in OpenBLAS
-- TODO: Check with Accelerate (OSX), too
-- https://developer.apple.com/library/mac/documentation/Darwin/Reference/Manpages/man7/Accelerate.7.html
-- https://github.com/xianyi/OpenBLAS/wiki/faq#multi-threaded
local util   = require'util'
local cutil  = require'cutil'
local udp    = require'udp'
local png    = require'png'
local jpeg   = require'jpeg'
jpeg.set_quality( 95 )

-- Channels
local mesh_udp_ch, mesh_tcp_ch, mesh_ch, lidar_ch
local channel_polls
--milliseconds
local channel_timeout = 100

-- Compression
local j_compress = jpeg.compressor('gray')
	
-- Setup metadata and tensors for a lidar mesh
local reading_per_radian, scan_resolution, fov_resolution
local mesh, mesh_byte, mesh_adj, scan_angles, offset_idx
-- LIDAR properties
local n, res, fov = 1081, 1, 270
local current_scanline, current_direction
local function setup_mesh()
  -- Find the resolutions
  scan_resolution = math.ceil(
		metadata.density * math.abs(metadata.scanlines[2]
			-metadata.scanlines[1])
	)
  -- Set our resolution
	-- NOTE: This has been changed in the lidar msgs...
  reading_per_radian = (n-1)/(270*DEG_TO_RAD)
  fov_resolution = reading_per_radian * math.abs(metadata.fov[2]-metadata.fov[1])
  fov_resolution = math.ceil(fov_resolution)
	-- Find the offset for copying lidar readings into the mesh
  -- if fov is from -fov/2 to fov/2 degrees, then offset_idx is zero
  -- if fov is from 0 to fov/2 degrees, then offset_idx is sensor_width/2
  local fov_offset = (n-1)/2+math.ceil( reading_per_radian*metadata.fov[1] )
  offset_idx   = math.floor(fov_offset)
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
	mesh_adj  = torch.FloatTensor( scan_resolution, fov_resolution )
  mesh_byte = torch.ByteTensor( scan_resolution, fov_resolution )
  -- Save the exact actuator angles of every scan
  scan_angles  = torch.DoubleTensor( scan_resolution ):zero()
end

------------------------------
-- Data copying helpers
-- Convert a pan angle to a column of the chest mesh image
local function angle_to_scanlines( rad )
  -- Get the most recent direction the lidar was moving
  local prev_scanline = current_scanline
  -- Get the metadata for calculations
  local start = metadata.scanlines[1]
  local stop  = metadata.scanlines[2]
  local ratio = (rad-start)/(stop-start)
  -- Round...? Why??
	-- TODO: Make this simpler/smarter
  local scanline = math.floor(ratio*scan_resolution+.5)
  -- Return a bounded value
  scanline = math.max( math.min(scanline, scan_resolution), 1 )
  --SJ: I have no idea why, but this fixes the scanline tilting problem
  if current_direction then
    if current_direction<0 then        
      scanline = math.max(1,scanline-1)
    else
      scanline = math.min(scan_resolution,scanline+1)
    end
  end
  -- Save in our table
  current_scanline = scanline
	-- Initialize if no previous scanline
  -- If not moving, assume we are staying in the previous direction
	if not prev_scanline then return {scanline} end
  -- Grab the most recent scanline saved in the mesh
  local prev_direction = current_direction
  if not prev_direction then
    current_direction = 1
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
  current_direction = direction
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

local function lidar_cb(s)
	-- NOTE: Ensure that we are able to run the cb fast enough
	-- For some reason the while loop is bad with luajit...
  local ch = poller.lut[s]
	-- Send message to a thread
	local data, has_more = ch:receive()
	-- If no msg, then process
	--if not data then break end
	-- Must have a pair with the range data
	assert(has_more,"metadata and not lidar ranges!")
	local ranges, has_more = ch:receive()
	local meta = mp.unpack(data)
	-- Update the points
	if meta.n~=n then
		print(metadata.name,'Properties',n,'=>',meta.n)
		n = meta.n
		setup_mesh()
		current_direction, current_scanline = nil, nil
	end
	-- Save the rpy of the body
	metadata.rpy = meta.rpy
	-- Save the latest lidar timestamp
	metadata.t = meta.t
  -- Save the body pose info
  local px, py, pa = unpack(meta.pose)
  -- Insert into the correct column
  local scanlines = angle_to_scanlines( meta.angle )
  -- Update each outdated scanline in the mesh
  for _,line in ipairs(scanlines) do
		-- Copy lidar readings to the torch object for fast modification
		-- TODO: This must change...
		-- Use string2storage? We only want one copy operation...
		-- Place into storage
		cutil.string2tensor(ranges, mesh:select(1,line), mesh:size(2), offset_idx)
		-- Save the pan angle
		scan_angles[line] = meta.angle
    -- Save the pose
		--[[
    chest.meta.posex[line],
    chest.meta.posey[line],
    chest.meta.posez[line]=
    px, py, pz
		--]]
  end
end

local function send_mesh(is_reliable)
	-- TODO: Somewhere check that far>near
  local near, far = unpack(metadata.dynrange)
  -- Enhance the dynamic range of the mesh image
  mesh_adj:copy(mesh.mesh):add( -near )
  mesh_adj:mul( 255/(far-near) )
  -- Ensure that we are between 0 and 255
  mesh_adj[torch.lt(mesh_adj,0)] = 0
  mesh_adj[torch.gt(mesh_adj,255)] = 255
  mesh_byte:copy( mesh_adj )
  -- Compression
  local c_mesh 
  local dim = mesh_byte:size()
  if metadata.c=='jpeg' then
    -- jpeg
		c_mesh = j_compress:compress(mesh_byte:storage():pointer(), dim[2], dim[1])
  elseif metadata.c=='png' then
    -- png
    mesh.meta.c = 'png'
    c_mesh = png.compress(mesh_byte:storage():pointer(), dim[2], dim[1], 1)
  else
    -- raw data?
		-- Maybe needed for sending a mesh to another process
    return
  end
	-- NOTE: Metadata should be packed only when it changes...
	local metapack = mp.pack(metadata)
	if is_reliable then
		local ret = mesh_tcp_ch:send{metapack,c_mesh}
	else
		local ret, err = mesh_udp_ch:send(metapack..c_mesh)
		if err then return err end
	end
end

local child_cmds = {
	reliable = function() send_mesh(true) end,
	unreliable = send_mesh,
}

-- Initial setup of the mesh from metadata
setup_mesh()
-- Data sending channels
mesh_tcp_ch = simple_ipc.new_publisher(Config.net.reliable_mesh)
mesh_udp_ch = udp.new_sender(Config.net.operator.wired, Config.net.mesh)
-- Poll for the lidar and master thread info
lidar_ch = simple_ipc.new_subscriber(metadata.name)
lidar_ch.callback = lidar_cb
pair_ch.callback = function(s)
	local ch = poller.lut[s]
	local data, has_more = ch:receive()
	local fun, ret = child_cmds[data]
	-- If a function, then execute it
	if fun then ret = fun() end
	-- Send anything back to the master thread if needed
	if ret then ch:send(ret) end
end

local wait_channels = {}
table.insert(wait_channels,lidar_ch)
table.insert(wait_channels,pair_ch)
poller = simple_ipc.wait_on_channels( wait_channels )
print('Child | Start poll')
poller:start()
