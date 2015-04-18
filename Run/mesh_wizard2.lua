#!/usr/bin/env luajit
local ENABLE_LOG = false
-- Mesh Wizard for Team THOR
-- Accumulate lidar readings into an image for mesh viewing
-- (c) Stephen McGill, Seung Joon Yi, 2013, 2014
dofile'../include.lua'
local libMesh = require'libMesh'
local torch = require'torch'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local vector = require'vector'
local Body = require'Body'
require'vcm'
require'hcm'

local mesh_ch, mesh_udp_ch
local t_send_mesh = -math.huge
local libLog, logger
local mesh0, mesh1
local mag_sweep0, t_sweep0, ranges_fov0
local mag_sweep1, t_sweep1, ranges_fov1

local function check_send_mesh()
	local request = false
	local t = Body.get_time()
	local n_open = hcm.get_network_open()
	local t_open
	if n_open==1 then
		t_open = hcm.get_network_topen()
		if t_open - t_send_mesh > 0.5 then request = true end
	end

	if t-t_send_mesh>t_sweep0 then request = true end
	if not request then return end

	--if not mesh0 then return end
	local metadata = mesh0.metadata
	mesh0:dynamic_range(vcm.get_mesh0_dynrange())
	local c_mesh = mesh0:get_png_string2()
	metadata.c = 'png'

	-- Send away
	local meta = mpack(metadata)
	if mesh_ch then mesh_ch:send{meta, c_mesh} end
	if mesh_udp_ch then
		local ret, err = mesh_udp_ch:send(meta..c_mesh)
		--print('Mesh | Sent UDP', err or 'successfully')
	end

	t_send_mesh = t

	-- Log
	if ENABLE_LOG then
		local raw_str = mesh0:get_raw_string()
		metadata.rsz = #raw_str
		logger:record(metadata, raw_str)
		if logger.n % 100 == 0 then
			logger:stop()
			logger = libLog.new('mesh', true)
			print('Open new log!')
		end
	end

	if not mesh1 then return end
	mesh1:dynamic_range(vcm.get_mesh1_dynrange())
	mesh1:save('/tmp/raw.log', '/tmp/byte.log')

end

local function entry()
	local stream = Config.net.streams.mesh
	local operator = Config.net.use_wireless and Config.net.operator.wireless or Config.net.operator.wired
	mesh_udp_ch = stream.udp and si.new_sender(operator, stream.udp)
	mesh_ch = stream.sub and si.new_publisher(stream.sub)
	if ENABLE_LOG then
		libLog = require'libLog'
		logger = libLog.new('mesh', true)
	end
end


local function update(meta, ranges)
	-- Check shared parameters
	if meta.id=='lidar0' then
		local mag_sweep, t_sweep = unpack(vcm.get_mesh0_sweep())
		mag_sweep = math.min(math.max(mag_sweep, 10 * DEG_TO_RAD), math.pi)
		t_sweep = math.min(math.max(t_sweep, 1), 20)
		local ranges_fov = vcm.get_mesh0_fov()
		-- Check if updated parameters
		if not mesh0 or mag_sweep~=mag_sweep0 or t_sweep~=t_sweep0 or ranges_fov~=ranges_fov0 then
			-- lidar is 40Hz
			local t_scan = 1 / 40
			mag_sweep0 = mag_sweep
			t_sweep0 = t_sweep
			ranges_fov0 = ranges_fov
			mesh0 = libMesh.new('mesh0', {
				n_lidar_returns = meta.n,
				lidar_resolution = meta.res,
				rfov = ranges_fov0,
				sfov = {-mag_sweep / 2, mag_sweep / 2},
				n_scanlines = math.floor(t_sweep / t_scan + 0.5),
			})
			print('Mesh0 | Updated containers')
		end
		mesh0:add_scan(meta.angle, ranges, meta)
	elseif meta.id=='lidar1' then
		local mag_sweep, t_sweep = unpack(vcm.get_mesh1_sweep())
		mag_sweep = math.min(math.max(mag_sweep, 10 * DEG_TO_RAD), math.pi)
		t_sweep = math.min(math.max(t_sweep, 1), 20)
		local ranges_fov = vcm.get_mesh1_fov()
		-- Check if updated parameters
		if not mesh1 or mag_sweep~=mag_sweep1 or t_sweep~=t_sweep1 or ranges_fov~=ranges_fov1 then
			-- lidar is 40Hz
			local t_scan = 1 / 40
			mag_sweep1 = mag_sweep
			t_sweep1 = t_sweep
			ranges_fov1 = ranges_fov
			mesh1 = libMesh.new('mesh1', {
				n_lidar_returns = meta.n,
				lidar_resolution = meta.res,
				rfov = ranges_fov1,
				sfov = {0, mag_sweep},
				n_scanlines = math.floor(t_sweep / t_scan + 0.5),
			})
			print('Mesh1 | Updated containers')
		end
		mesh1:add_scan(meta.angle[2], ranges, meta)
	end

	-- Check for sending out on the wire
	-- TODO: This *should* be from another ZeroMQ event, in case the lidar dies
	check_send_mesh()

end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=entry, update=update, exit=nil}
end

local poller
local function cb(skt)
	local idx = poller.lut[skt]
	local mdata, ranges = unpack(skt:recv_all())
	local meta = munpack(mdata)
	update(meta, ranges)
end

local lidar0_ch = si.new_subscriber'lidar0'
local lidar1_ch = si.new_subscriber'lidar1'

lidar0_ch.callback = cb
lidar1_ch.callback = cb
poller = si.wait_on_channels({lidar_ch})

-- Cleanly exit on Ctrl-C
local signal = require'signal'.signal
local running = true
local function shutdown()
	io.write('Shutdown!\n')
	poller:stop()
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

poller:start()

if ENABLE_LOG then
	logger:stop()
end
