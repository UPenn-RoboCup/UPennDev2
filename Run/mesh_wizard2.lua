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

local mesh

-- Open up channels to send/receive data
local operator
if Config.net.use_wireless then
	operator = Config.net.operator.wireless
else
	operator = Config.net.operator.wired
end
local stream = Config.net.streams.mesh
local mesh_udp_ch
local mesh_tcp_ch = stream.tcp and si.new_publisher(stream.tcp)
local mesh_ch = stream.sub and si.new_publisher(stream.sub)
-- Need UDP when competing
if Config.IS_COMPETING then
	print("UDP", operator, stream.udp)
	mesh_udp_ch = stream.udp and si.new_sender(operator, stream.udp)
end

local libLog, logger, nlog
if ENABLE_LOG then
	libLog = require'libLog'
	logger = libLog.new('mesh', true)
	nlog = 0
end

local function send_mesh(compression, dynrange)
	local near, far = unpack(dynrange)
	if near>far then
		print('Near greater than far...')
		return
	end
	print('sending...')

	local metadata = mesh.metadata
	mesh:dynamic_range()
	local c_mesh = mesh:get_png_string2()
	metadata.c = 'png'
	mesh:save('/tmp/raw.log', '/tmp/byte.log')
	print('dim', unpack(mesh.metadata.dim))

	--if true then return end

	-- Send away
	mesh_ch:send{mpack(metadata), c_mesh}
	mesh_tcp_ch:send{mpack(metadata), c_mesh}
	if mesh_udp_ch then
		local ret, err = mesh_udp_ch:send(mpack(metadata)..c_mesh)
		print('Mesh | Sent UDP', err or 'successfully')
	end
end

local t_send_mesh = -math.huge
local function check_send_mesh()
	local net = vcm.get_mesh_net()
	local request, comp = unpack(net)
	local t_check = Body.get_time()
	local n_open = hcm.get_network_open()
	local t_open
	if n_open==1 then
		t_open = hcm.get_network_topen()
		if t_open - t_send_mesh > 0.5 then request = 1 end
	end
	if request==0 then return end
	local dynrange = vcm.get_mesh_dynrange()
	send_mesh('png', dynrange)
	t_send_mesh = t_check

	-- Reset the request
	net[1] = 0
	vcm.set_mesh_net(net)


	-- Log
	-- Do the logging if we wish
	if ENABLE_LOG then
		metadata.rsz = mesh:nElement() * ffi.sizeof'float'
		logger:record(metadata, mesh:data(), metadata.rsz)
		nlog = nlog + 1
		print(request, "# mesh logs: "..nlog, metadata.rsz)
		if nlog % 100 == 0 then
			logger:stop()
			logger = libLog.new('mesh', true)
			print('Open new log!')
		end
	end

end

local function update(meta, ranges)
	-- Check shared parameters
	local mag_sweep0, t_sweep0 = unpack(vcm.get_mesh_sweep())
	local ranges_fov0 = vcm.get_mesh_fov()
	mag_sweep0 = math.min(math.max(mag_sweep0, 10 * DEG_TO_RAD), math.pi)
	t_sweep0 = math.min(math.max(t_sweep0, 1), 20)
	-- Check if updated parameters
	if not mesh or mag_sweep~=mag_sweep0 or t_sweep~=t_sweep0 or ranges_fov~=ranges_fov0 then
		-- lidar is 40Hz
		local t_scan = 1 / 40
		mag_sweep = mag_sweep0
		t_sweep = t_sweep0
		ranges_fov = ranges_fov0
		mesh = libMesh.new('mesh0', {
				n_lidar_returns = meta.n,
				lidar_resolution = meta.res,
				rfov = ranges_fov0,
				sfov = {-mag_sweep / 2, mag_sweep / 2},
				n_scanlines = math.floor(t_sweep / t_scan + 0.5)
			})
		print('Mesh | Updated containers')
	end

	mesh:add_scan(meta, ranges)

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
end
local poller = si.wait_on_channels({lidar_ch})

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
