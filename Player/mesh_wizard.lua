-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------
dofile'../include.lua'
-- Going to be threading this
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local torch = require'torch'
local util = require'util'
local p_compress = require'png'.compress
local j_compress = require'jpeg'.compressor'gray'

-- Grab the metadata for this lidar mesh
local metadata, lidar_id
if not arg or not arg[1] then
	-- TODO: Find the next available camera
	lidar_id = 1
	metadata = Config.lidar[1]
else
	lidar_id = tonumber(arg[1])
	if lidar_id then
		metadata = assert(Config.lidar[lidar_id], 'Bad LIDAR ID')
	else
		for id, c in ipairs(Config.lidar) do
			if arg[1] == c.name then
				lidar_id = id
				metadata = c
				break
			end
		end
		assert(metadata, 'Bad camera name')
	end
end

-- Open up channels to send/receive data
local mesh_tcp_ch = si.new_publisher(Config.net.reliable_mesh)
local mesh_udp_ch = si.new_sender(Config.net.operator.wired, Config.net.mesh)

local metadata = {
	name = v,
	fov = {-60*DEG_TO_RAD,60*DEG_TO_RAD},
	scanlines = {-45*DEG_TO_RAD,45*DEG_TO_RAD},
	density = 10*RAD_TO_DEG, -- #scanlines per radian on actuated motor
	dynrange = {.1,1}, -- Dynamic range of depths when compressing
	c = 'jpeg', -- Type of compression
	t = 0,
}

-- Setup metadata and tensors for a lidar mesh
local reading_per_radian, scan_resolution, fov_resolution
local mesh, mesh_byte, mesh_adj, scan_angles, offset_idx
local current_scanline, current_direction
local function setup_mesh(meta)
	local n, res = meta.n, meta.res
	local fov = n * res
  -- Find the resolutions
  scan_resolution = math.ceil(
		metadata.density * math.abs(metadata.scanlines[2]
			-metadata.scanlines[1])
	)
  -- Set our resolution
	-- NOTE: This has been changed in the lidar msgs...
  reading_per_radian = (n-1)/(fov*DEG_TO_RAD)
  fov_resolution = reading_per_radian * math.abs(metadata.fov[2]-metadata.fov[1])
  fov_resolution = math.ceil(fov_resolution)
	-- Find the offset for copying lidar readings into the mesh
  -- if fov is from -fov/2 to fov/2 degrees, then offset_idx is zero
  -- if fov is from 0 to fov/2 degrees, then offset_idx is sensor_width/2
  local fov_offset = (n-1)/2+math.ceil( reading_per_radian*metadata.fov[1] )
  offset_idx = math.floor(fov_offset)
  -- TODO: Pose information for each scanline
  -- In-memory mesh
  mesh = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  -- Mesh buffers for compressing and sending to the user
	mesh_adj  = torch.FloatTensor( scan_resolution, fov_resolution )
  mesh_byte = torch.ByteTensor( scan_resolution, fov_resolution )
  -- Save the exact actuator angles of every scan
  scan_angles  = torch.DoubleTensor( scan_resolution ):zero()
	-- No direction information yet
	current_direction, current_scanline = nil, nil
end

------------------------------
-- Data copying helpers
-- Convert a pan angle to a column of the chest mesh image
local function angle_to_scanlines(rad)
  -- Get the most recent direction the lidar was moving
  local prev_scanline = current_scanline
  -- Get the metadata for calculations
  local start, stop = unpack(metadata.scanlines)
  local ratio = (rad-start)/(stop-start)
  -- Round...? Why??
	-- TODO: Make this simpler/smarter
  local scanline = math.floor(ratio*scan_resolution+.5)
  -- Return a bounded value
  scanline = math.max( math.min(scanline, scan_resolution), 1 )
  --SJ: I have no idea why, but this fixes the scanline tilting problem
  if current_direction and current_direction<0 then
		scanline = math.max(1, scanline-1)
	else
		scanline = math.min(scan_resolution, scanline + 1)
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

local function update(meta, ranges)
	-- Update the points
	if not mesh then setup_mesh(meta) end
	-- Save the latest lidar timestamp
	metadata.t = meta.t
	-- TODO: Save the rpy of the body
	--metadata.rpy = meta.rpy
  -- TODO: Save the body pose info
  --local px, py, pa = unpack(meta.pose)
  -- Insert into the correct column
  local scanlines = angle_to_scanlines( meta.angle )
  -- Update each outdated scanline in the mesh
  for _,line in ipairs(scanlines) do
		-- Copy lidar readings to the torch object for fast modification
		cutil.string2tensor(ranges, mesh:select(1,line), mesh:size(2), offset_idx)
		-- Save the pan angle
		scan_angles[line] = meta.angle or scan_angles[line] or 0
    -- TODO: Save the pose into each scanline
  end
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=nil, update=update, exit=nil}
end

local lidar_ch = si.new_subscriber(metadata.name)
function lidar_ch.callback(skt)
	local mdata, ranges = unpack(skt:recv_all())
	local meta = munpack(mdata)
	update(meta, ranges)
end

local function send_mesh(use_reliable)
	-- TODO: Somewhere check that far>near
  local near, far = unpack(metadata.dynrange)
  -- Enhance the dynamic range of the mesh image
  mesh_adj:copy(mesh.mesh):add(-near)
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
    c_mesh = p_compress(mesh_byte:storage():pointer(), dim[2], dim[1], 1)
  else
    -- raw data?
		-- Maybe needed for sending a mesh to another process
    return
  end
	-- NOTE: Metadata should be packed only when it changes...
	if use_reliable then
		mesh_tcp_ch:send{mpack(metadata), c_mesh}
	else
		local ret, err = mesh_udp_ch:send(mpack(metadata)..c_mesh)
		if err then return err end
	end
end

-- TODO: Listen for mesh requests on a separate channel (mesh_ch as REP/REQ)
si.wait_on_channels({lidar_ch}):start()
