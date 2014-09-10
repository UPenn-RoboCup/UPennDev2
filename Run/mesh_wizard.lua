#!/usr/bin/env luajit
-- Mesh Wizard for Team THOR
-- Accumulate lidar readings into an image for mesh viewing
-- (c) Stephen McGill, Seung Joon Yi, 2013, 2014
dofile'../include.lua'
local ffi = require'ffi'
local torch = require'torch'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local p_compress = require'png'.compress
local j_compress = require'jpeg'.compressor'gray'
local vector = require'vector'
require'vcm'

-- Shared with LidarFSM
-- t_sweep: Time (seconds) to fulfill scan angles in one sweep
-- mag_sweep: How much will we sweep over
-- ranges_fov: In a single scan, which ranges to use
local t_sweep, mag_sweep, ranges_fov
-- NOTE: The following is LIDAR dependent
local t_scan = 1 / 40 -- Time to gather returns

-- Open up channels to send/receive data
local operator
if Config.net.use_wireless then
	operator = Config.net.operator.wireless
else
	operator = Config.net.operator.wired
end
local stream = Config.net.streams['mesh']
local mesh_tcp_ch = stream.tcp and si.new_publisher(stream.tcp, operator)
local mesh_udp_ch = stream.udp and si.new_sender(operator, stream.udp)
local mesh_ch = stream.sub and si.new_publisher(stream.sub)
print("OPERATOR", operator, stream.udp)

local metadata = {
	name = 'mesh0',
	t = 0,
}

-- Setup tensors for a lidar mesh
local mesh, mesh_byte, mesh_adj, scan_angles, offset_idx
local n_scanlines
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
	print("n_returns", n_returns, max_view, min_view, res)
	-- Check the number of scanlines in each mesh
	-- Indexed by the actuator angle
	-- Depends on the speed we use
	n_scanlines = math.floor(t_sweep / t_scan + 0.5)
  -- In-memory mesh
  mesh = torch.FloatTensor(n_scanlines, n_returns):zero()
  -- Mesh buffers for compressing and sending to the user
	mesh_adj  = torch.FloatTensor(n_scanlines, n_returns)
  mesh_byte = torch.ByteTensor(n_scanlines, n_returns)
  -- Data for each scanline
	if TORCH_SCANLINE_INFO then
		scan_angles = torch.DoubleTensor(n_scanlines):zero()
		scan_x = torch.DoubleTensor(n_scanlines):zero()
		scan_y = torch.DoubleTensor(n_scanlines):zero()
		scan_a = torch.DoubleTensor(n_scanlines):zero()
	else
		scan_angles = vector.zeros(n_scanlines)
		scan_x = vector.zeros(n_scanlines)
		scan_y = vector.zeros(n_scanlines)
		scan_a = vector.zeros(n_scanlines)
	end
	-- Metadata for the mesh
	metadata.rfov = ranges_fov
	metadata.sfov = {-mag_sweep / 2, mag_sweep / 2}
	metadata.a = scan_angles
	metadata.px = scan_x
	metadata.py = scan_y
	metadata.pa = scan_a
	-- TODO: Add IMU for body orientation
end

-- Convert a pan angle to a column of the chest mesh image
local scanline, direction
local function angle_to_scanlines(rad)
	-- Find the scanline
  local ratio = (rad + mag_sweep / 2) / mag_sweep
  local scanline_now = math.floor(ratio*n_scanlines+.5)
  scanline_now = math.max(math.min(scanline_now, n_scanlines), 1)
	local prev_scanline = scanline or scanline_now
	scanline = scanline_now
	-- Find the direction
	local direction_now = vcm.get_mesh_state()
	local prev_direction = direction or direction_now
	direction = direction_now
	-- If unknown, then only populate a single scanline
	if direction == 0 then return {scanline} end
  -- Find the set of scanlines for copying the lidar reading
  local scanlines = {}
	-- No direction change
  if direction==prev_direction then
    -- Fill all lines between previous and now
    for s=prev_scanline+direction,scanline,direction do table.insert(scanlines,s) end
		return scanlines
	end
	-- Changed directions, so populate the borders, too
	if direction > 0 then
		local fill_line = math.max(prev_scanline - 1, scanline)
		for s=1,fill_line do table.insert(scanlines, s) end
	else
		local fill_line = math.min(prev_scanline + 1, scanline)
		for s=fill_line,n_scanlines do table.insert(scanlines, s) end
	end
  return scanlines
end

local function send_mesh(destination, compression, dynrange)
  local near, far = unpack(dynrange)
	if near>far then
		print('Near greater than far...')
		return
	end
  -- Enhance the dynamic range of the mesh image
  mesh_adj:copy(mesh):add(-near)
  mesh_adj:mul(255/(far-near))
  -- Ensure that we are between 0 and 255
  mesh_adj[torch.lt(mesh_adj,0)] = 0
  mesh_adj[torch.gt(mesh_adj,255)] = 255
  mesh_byte:copy(mesh_adj)
  -- Compression
  local c_mesh
  if compression=='jpeg' then
		c_mesh = j_compress:compress(mesh_byte)
  elseif compression=='png' then
    c_mesh = p_compress(mesh_byte)
  else
    -- raw data?
		-- Maybe needed for sending a mesh to another process
		print('No raw sending support...')
    return
  end
	-- Update relevant metadata
	metadata.c = compression
	metadata.dr = dynrange
	-- Send away
	if type(destination)=='string' then
		local f_img = io.open(destination..'.'..metadata.c, 'w')
		local f_meta = io.open(destination..'.meta', 'w')
		f_img:write(c_mesh)
		f_img:close()
		f_meta:write(mpack(metadata))
		f_meta:close()
		print('Mesh | Wrote local')
  elseif IS_WEBOTS and mesh_ch then
    mesh_ch:send{mpack(metadata), c_mesh}
    print('Mesh | Sent PUB')
	elseif destination then
		mesh_tcp_ch:send{mpack(metadata), c_mesh}
		print('Mesh | Sent TCP')
	else
		local ret, err = mesh_udp_ch:send(mpack(metadata)..c_mesh)
		print('Mesh | Sent UDP', err or 'successfully')
	end
end

local function check_send_mesh()
	local net = vcm.get_mesh_net()
	local request, destination, compression = unpack(net)
	if request==0 then return end
	local dynrange = vcm.get_mesh_dynrange()
	send_mesh(destination==1, compression==1 and 'png' or 'jpeg', dynrange)
	-- Reset the request
	net[1] = 0
	vcm.set_mesh_net(net)
end

local function update(meta, ranges)
	-- Check shared parameters
	local mag_sweep0, t_sweep0 = unpack(vcm.get_mesh_sweep())
	local ranges_fov0 = vcm.get_mesh_fov()
	mag_sweep0 = math.min(math.max(mag_sweep0, 10 * DEG_TO_RAD), math.pi)
	t_sweep0 = math.min(math.max(t_sweep0, 1), 20)
	-- Check if updated parameters
	if not mesh or mag_sweep~=mag_sweep0 or t_sweep~=t_sweep0 or ranges_fov~=ranges_fov0 then
		mag_sweep = mag_sweep0
		t_sweep = t_sweep0
		ranges_fov = ranges_fov0
		setup_mesh(meta)
		print('Mesh | Updated containers')
	end
  -- Find the scanline indices
	local rad_angle = meta.angle
  local scanlines = angle_to_scanlines(rad_angle)
	local byte_sz = mesh:size(2) * ffi.sizeof'float'
	local float_ranges = ffi.cast('float*', ranges)
	local dest
  for _,line in ipairs(scanlines) do
		if line >= 1 and line<=n_scanlines then
			dest = mesh:select(1, line) -- NOTE: must be contiguous
			ffi.copy(dest:data(), float_ranges + offset_idx, byte_sz)
			-- Save the pan angle
			scan_angles[line] = rad_angle
			-- TODO: Save the pose
		end
  end
	-- Check for sending out on the wire
	-- TODO: This *should* be from another ZeroMQ event, in case the lidar dies
	check_send_mesh()
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=nil, update=update, exit=nil}
end

local lidar_ch = si.new_subscriber'lidar0'
function lidar_ch.callback(skt)
	local mdata, ranges = unpack(skt:recv_all())
	local meta = munpack(mdata)
	update(meta, ranges)
	print("hey")
end

si.wait_on_channels({lidar_ch}):start()