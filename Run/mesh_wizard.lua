#!/usr/bin/env luajit
local ENABLE_LOG = false
-- Mesh Wizard for Team THOR
-- Accumulate lidar readings into an image for mesh viewing
-- (c) Stephen McGill, Seung Joon Yi, 2013, 2014
dofile'../include.lua'
local libMesh = require'libMesh'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local Body = require'Body'
require'vcm'
require'hcm'

local mesh0_udp_ch, mesh1_udp_ch
local mesh0_tcp_ch, mesh1_tcp_ch
local mesh0_ch, mesh1_ch
local t_send = -math.huge
local libLog, logger
local mesh0, mesh1
local mag_sweep0, t_sweep0, ranges_fov0
local mag_sweep1, t_sweep1, ranges_fov1
--
local hz_outdoor_send = 1
local dt_outdoor_send = 1/hz_outdoor_send
--
local hz_indoor_send = 3
local dt_indoor_send = 1/hz_indoor_send

local function check_send_mesh()
	local is_outdoors = hcm.get_network_indoors()==0
	local is_indoors = not is_outdoors
	--
	local t = Body.get_time()
	local dt_send0 = t - t_send

	if is_outdoors and dt_send0 < dt_outdoor_send then return end
	if is_indoors and dt_send0 < dt_indoor_send then return end
	t_send = t
	--print('Mesh | Sending', dt_send0)
	if mesh0 then
		local metadata = mesh0.metadata
		metadata.t = t
		local raw_msg = {mpack(metadata), mesh0:get_raw_string()}
		metadata.c = 'raw'

		-- Send away
		if mesh0_ch then
			--metadata.c = 'raw'
			mesh0_ch:send(raw_msg)
			--metadata.c = 'png'
			--mesh0_ch:send{mpack(metadata), c_mesh}
		end
		if mesh0_tcp_ch then
			--metadata.c = 'raw'
			mesh0_tcp_ch:send(raw_msg)
			--metadata.c = 'png'
			--mesh0_ch:send{mpack(metadata), c_mesh}
		end
		if mesh0_udp_ch then
			--mesh0:dynamic_range(vcm.get_mesh0_dynrange())
			--local c_mesh = mesh0:get_png_string2()
			--metadata.c = 'png'
			--local meta = mpack(metadata)
			--local ret, err = mesh0_udp_ch:send(meta..c_mesh)
			local ret, err = mesh0_udp_ch:send(table.concat(raw_msg))
			--print('Mesh0 | Sent UDP', unpack(ret))
		end
	end

	if mesh1 then
		local metadata = mesh1.metadata
		metadata.t = t
		metadata.c = 'raw'
		local raw_msg = {mpack(metadata), mesh1:get_raw_string()}
		--mesh1:dynamic_range(vcm.get_mesh1_dynrange())
		--local c_mesh = mesh1:get_png_string2()
		--metadata.c = 'png'
		--local png_msg = {mpack(metadata), c_mesh}

		-- Send away
		if mesh1_ch then
			mesh1_ch:send(raw_msg)
		end
		if mesh1_tcp_ch then
			mesh1_tcp_ch:send(raw_msg)
		end
		if mesh1_udp_ch then
			local ret, err = mesh1_udp_ch:send(table.concat(raw_msg))
			--print('Mesh1 | Sent UDP', unpack(ret))
		end
	end

	-- Log
	if logger0 and mesh0 then
		local raw_str = mesh0:get_raw_string()
		mesh0.metadata.rsz = #raw_str
		logger0:record(mesh0.metadata, raw_str)
		if logger0.n % 10 == 0 then
			logger0:stop()
			logger0 = libLog.new('mesh0', true)
			print('Mesh0 | Open new log!')
		end
	end

	-- Log
	if logger1 and mesh1 then
		local raw_str = mesh1:get_raw_string()
		mesh1.metadata.rsz = #raw_str
		logger1:record(mesh1.metadata, raw_str)
		if logger1.n % 10 == 0 then
			logger1:stop()
			logger1 = libLog.new('mesh1', true)
			print('Mesh1 | Open new log!')
		end
	end

end

local function entry()
	local stream0 = Config.net.streams.mesh0
	local stream1 = Config.net.streams.mesh1
	local operator = Config.net.operator.wired
	--
	mesh0_udp_ch = stream0.udp and si.new_sender(operator, stream0.udp)
	mesh0_ch = stream0.sub and si.new_publisher(stream0.sub)
	mesh0_tcp_ch = stream0.tcp and si.new_publisher(stream0.tcp)
	--
	mesh1_udp_ch = stream1.udp and si.new_sender(operator, stream1.udp)
	mesh1_ch = stream1.sub and si.new_publisher(stream1.sub)
	mesh1_tcp_ch = stream1.tcp and si.new_publisher(stream1.tcp)
	--
	if ENABLE_LOG then
		libLog = require'libLog'
		logger0 = libLog.new('mesh0', true)
		logger1 = libLog.new('mesh1', true)
	end
end


local function update(meta, ranges)
--require'util'.ptable(meta)
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
			local props0 = {
				n_lidar_returns = meta.n,
				lidar_resolution = meta.res,
				rfov = ranges_fov0,
				sfov = {-mag_sweep / 2, mag_sweep / 2},
				n_scanlines = math.floor(t_sweep / t_scan + 0.5),
			}
			mesh0 = libMesh.new('mesh0', props0)
			print('Mesh0 | Updated containers')
			require'util'.ptable(props0)
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
	if IS_WEBOTS then
		check_send_mesh()
	end
end

local function exit()
	if ENABLE_LOG then
		logger0:stop()
		logger1:stop()
	end
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=entry, update=update, exit=exit}
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
poller = si.wait_on_channels({lidar0_ch, lidar1_ch})

-- Cleanly exit on Ctrl-C
local signal = require'signal'.signal
local running = true
local function shutdown()
	io.write('Shutdown!\n')
	poller:stop()
	running = false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

-- Timeout in milliseconds (8 Hz)
local TIMEOUT = 1e3 / 8
entry()
--poller:start()
while running do
	npoll = poller:poll(TIMEOUT)
	check_send_mesh()
end
exit()
