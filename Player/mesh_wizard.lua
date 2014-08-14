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

-- Shared with LidarFSM
local t_sweep = 5 -- Time (seconds) to fulfill scan angles in one sweep
local mag_sweep = 90 * DEG_TO_RAD -- How much will we sweep over
--
local ranges_fov = {-60*DEG_TO_RAD, 60*DEG_TO_RAD} -- In a single scan, which ranges to use
--
local t_scan = 1 / 40 -- Time to gather returns
local half_mag_sweep = mag_sweep / 2

-- Open up channels to send/receive data
-- Who to send to
local operator
if Config.net.use_wireless then
	operator = Config.net.operator.wireless
else
	operator = Config.net.operator.wired_broadcast
end
local stream = Config.net.streams['mesh']
local mesh_tcp_ch = si.new_publisher(stream.tcp, operator)
local mesh_udp_ch = si.new_sender(operator, stream.udp)

local metadata = {
	name = v,
	dynrange = {.1, 1}, -- Dynamic range of depths when compressing
	c = 'png', -- Type of compression
	t = 0,
}

-- Setup metadata and tensors for a lidar mesh
local mesh, mesh_byte, mesh_adj, scan_angles, offset_idx
local current_scanline, current_direction
local n_returns, n_scanlines
local function setup_mesh(meta)
	local n, res = meta.n, meta.res
	local fov = n * res
	-- Check that we can access enough FOV
	local min_view, max_view = unpack(ranges_fov)
	assert(fov > max_view-min_view, 'Not enough FOV available')
	-- Find the offset for copying lidar readings into the mesh
  -- if fov is from -fov/2 to fov/2 degrees, then offset_idx is zero
  -- if fov is from 0 to fov/2 degrees, then offset_idx is sensor_width/2
  local fov_offset = min_view / res + n / 2
	-- Round the offset (0 based offset)
  offset_idx = math.floor(fov_offset + 0.5)
	-- Round to get the number of returns for each scanline
	n_returns = math.floor((max_view - min_view) / res + 0.5)
	-- Check the number of scanlines in each mesh
	-- Indexed by the actuator angle
	-- Depends on the speed we use
	n_scanlines = math.floor(t_sweep / t_scan + 0.5)
	--print('Tracking', n_scanlines, 'scanlines with ', n_returns, 'returns each')
  -- TODO: Pose information for each scanline
  -- In-memory mesh
  mesh = torch.FloatTensor(n_scanlines, n_returns):zero()
  -- Mesh buffers for compressing and sending to the user
	mesh_adj  = torch.FloatTensor(n_scanlines, n_returns)
  mesh_byte = torch.ByteTensor(n_scanlines, n_returns)
  -- Save the exact actuator angles of every scan
  scan_angles = torch.DoubleTensor(n_scanlines):zero()
	-- No direction information yet
	current_direction, current_scanline = nil, nil
end

------------------------------
-- Data copying helpers
-- Convert a pan angle to a column of the chest mesh image
local function angle_to_scanlines(rad)
  -- Get the most recent direction the lidar was moving
  local prev_scanline = current_scanline
  local ratio = (rad + half_mag_sweep)/mag_sweep
  local scanline = math.floor(ratio*n_scanlines+.5)
  -- Return a bounded value
  scanline = math.max( math.min(scanline, n_scanlines), 1 )
  --SJ: I have no idea why, but this fixes the scanline tilting problem
  if current_direction and current_direction<0 then
		scanline = math.max(1, scanline-1)
	else
		scanline = math.min(n_scanlines, scanline + 1)
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
      for s=start_line,n_scanlines do table.insert(scanlines,i) end
    else
      -- going away from end to 1
      local end_line = math.max(prev_scanline-1,scanline)
      for s=1,end_line do table.insert(scanlines,i) end        
    end
  end
  -- Return for populating
  return scanlines
end

local function send_mesh(destination)
	-- TODO: Somewhere check that far>near
  local near, far = unpack(metadata.dynrange)
  -- Enhance the dynamic range of the mesh image
  mesh_adj:copy(mesh):add(-near)
  mesh_adj:mul(255/(far-near))
  -- Ensure that we are between 0 and 255
  mesh_adj[torch.lt(mesh_adj,0)] = 0
  mesh_adj[torch.gt(mesh_adj,255)] = 255
  mesh_byte:copy(mesh_adj)
  -- Compression
  local c_mesh 
  local dim = mesh_byte:size()
  if metadata.c=='jpeg' then
		c_mesh = j_compress:compress(mesh_byte)
  elseif metadata.c=='png' then
    c_mesh = p_compress(mesh_byte)
  else
    -- raw data?
		-- Maybe needed for sending a mesh to another process
    return
  end
	-- NOTE: Metadata should be packed only when it changes...
	if type(destination)=='string' then
		local f_img = io.open(destination..'.'..metadata.c, 'w')
		local f_meta = io.open(destination..'.meta', 'w')
		f_img:write(c_mesh)
		f_img:close()
		f_meta:write(mpack(metadata))
		f_meta:close()
	elseif destination then
		mesh_tcp_ch:send{mpack(metadata), c_mesh}
	else
		local ret, err = mesh_udp_ch:send(mpack(metadata)..c_mesh)
		if err then return err end
	end
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
	local rad_angle = meta.angle
  local scanlines = angle_to_scanlines(rad_angle)
  -- Update each outdated scanline in the mesh
	local byte_sz = n_returns * ffi.sizeof'float'
  for _,line in ipairs(scanlines) do
		-- Copy lidar readings to the torch object for fast modification
		--cutil.string2tensor(ranges, mesh:select(1,line), mesh:size(2), offset_idx)
		-- Perform the copy. NOTE: mesh:select(1, line) must be contiguous!
		ffi.copy(mesh:select(1, line):data(), ffi.cast('float*', ranges) + offset_idx, byte_sz)
		-- Save the pan angle
		scan_angles[line] = rad_angle
    -- TODO: Save the pose into each scanline
  end
	-- DEBUG ONLY
	--[[
	t = unix.time()
	t0 = t0 or t
	if t-t0 > 1 then
		cnt = cnt and cnt + 1 or 1
		t0 = t
		print('SAVING!!', cnt)
		send_mesh('/tmp/mesh_'..cnt)
	end
	--]]
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

-- TODO: Listen for mesh requests on a separate channel (mesh_ch as REP/REQ)
si.wait_on_channels({lidar_ch}):start()
