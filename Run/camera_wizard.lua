#!/usr/bin/env luajit
local ENABLE_NET = true
local ENABLE_LOG = false

-----------------------------------
-- Camera manager
-- (c) Stephen McGill, 2014
-----------------------------------
dofile'../include.lua'
if type(arg)~='table' then IS_WEBOTS=true end
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local jpeg = require'jpeg'
local Body = require'Body'
local get_time = Body.get_time
local ok, ffi = pcall(require, 'ffi')

-- Grab the metadata for this camera
local metadata, camera_id
if type(arg)~='table' or not arg[1] then
	-- TODO: Find the next available camera
	camera_id = 1
	metadata = Config.camera[1]
else
	camera_id = tonumber(arg[1])
	if camera_id then
		metadata = assert(Config.camera[camera_id], 'Bad Camera ID')
	else
		for id, c in ipairs(Config.camera) do
			if arg[1] == c.name then
				camera_id = id
				metadata = c
				break
			end
		end
		assert(metadata, 'Bad camera name')
	end
end

-- Extract metadata information
local w = metadata.w
local h = metadata.h
local name = metadata.name
-- Who to send to
local operator
--operator = Config.net.operator.wireless
operator = Config.net.operator.wired
-- Network Channels/Streams
local camera_identifier = 'camera'..(camera_id-1)
local stream = Config.net.streams[camera_identifier]
local udp_ch = ENABLE_NET and stream and stream.udp and
	si.new_sender(operator, stream.udp)
local camera_ch = stream and stream.sub and si.new_publisher(stream.sub)
print('Camera | ', operator, camera_identifier, stream.udp, udp_ch)

-- Metadata for the operator for compressed image data
local c_yuyv = jpeg.compressor('yuyv')
c_yuyv:quality(metadata.quality)
c_yuyv:downsampling(metadata.downsampling)

-- LOGGING
local t_log = -math.huge
local LOG_INTERVAL = 1/5
local libLog, logger
if ENABLE_LOG then
	libLog = require'libLog'
	logger = libLog.new('yuyv', true)
end
--
local t0 = get_time()
local t_debug = 0
--
local hz_monitor = 1
local dt_monitor = 1/hz_monitor
local t_monitor = -math.huge

-- Figure out where the camera is on each frame
require'mcm'
require'wcm'
local pose_global = require'util'.pose_global
local transform6D = require'Transform'.transform6D
local rotY = require'Transform'.rotY
local rotZ = require'Transform'.rotZ
local trans = require'Transform'.trans
local from_rpy_trans = require'Transform'.from_rpy_trans
local flatten = require'Transform'.flatten
-- CoM to the Neck (32cm in z)
local tNeck = trans(unpack(Config.head.neckOffset))
-- Mounting of Camera from the neck axes
local tCamera = from_rpy_trans(unpack(metadata.mountOffset))
-- Give transformation of the camera
local function get_tf()
	local rpy = Body.get_rpy()
	local pose = wcm.get_robot_pose()
	local bh = mcm.get_walk_bodyHeight()
	local bo = mcm.get_status_bodyOffset()
	local qHead = Body.get_head_command_position()
	--local qHead = Body.get_head_position()
	local uComp = mcm.get_stance_uTorsoComp()
	uComp[3] = 0
	-- Poses with the compensation
	local torsoL = pose_global(uComp, bo)
	local torsoG = pose_global(torsoL, pose)
	-- Transform relative to the local body frame (on the ground between the feet)
	local tfTorsoLocal =
		transform6D{torsoL.x, torsoL.y, bh, rpy[1], rpy[2], torsoL.a}
	-- Transform relative to the global body frame (on the ground)
	local tfTorsoGlobal =
		transform6D{torsoG.x, torsoG.y, bh, rpy[1], rpy[2], torsoG.a}
	-- Transform relative to the center of mass
	local tfCom = tNeck * rotZ(qHead[1]) * rotY(qHead[2]) * tCamera
	-- Give the local and global transforms
	return tfTorsoLocal * tfCom, tfTorsoGlobal * tfCom
end

local camera_name = name..'_camera'
local function update(img, sz, cnt, t)

	local tfL, tfG = get_tf()
	local tfL_flat, tfG_flat = flatten(tfL), flatten(tfG)
	local metadata_camera = {
		head = Body.get_head_position(),
		w = w,
		h = h,
		t = t,
		c = 'yuyv',
		id = camera_name,
		--rpy = Body.get_rpy(),
		tfL16 = tfL_flat,
		tfG16 = tfG_flat
	}
	local img_str = ffi.string(img, sz)

	-- Send on the channel
	if camera_ch then
		--print(type(img), sz, #str)
		camera_ch:send({mpack(metadata_camera), img_str})
	end

	-- Log raw frames
	if logger and (t - t_log > LOG_INTERVAL) then
		t_log = t
		metadata_camera.rsz = #img_str
		logger:record(metadata_camera, img_str)
		if logger.n % 10 == 0 then
			print("# camera logs: ", logger.n)
			if logger.n % 100 == 0 then
				logger:stop()
				print('Open new log!')
				logger = libLog.new('yuyv', true)
			end
		end
	end

	-- Send debugging jpeg
	if udp_ch and (t - t_monitor > dt_monitor) then
		local c_img = c_yuyv:compress(img_str, w, h)
		local msg = {mpack({
			-- Required for rendering
			sz = #c_img,
			c = 'jpeg',
			-- Extra information
			t = t,
			id = camera_name,
			n = cnt,
		}), c_img}
		local udp_ret, udp_err = udp_ch:send(table.concat(msg))
		t_monitor = t
	end

end

-- If required from Webots, return the table
if ... and type(...)=='string' and not tonumber(...) then
	return {entry=nil, update=update, exit=nil}
end

-- Open the camera
print('Opening', metadata.dev)
local camera = require'uvc'.init(
	metadata.dev, w, h, metadata.format, 1, metadata.fps
)

local dev_raw
do
	local f = io.popen("ls -l "..metadata.dev)
	dev_raw = f:read("*all"):match'video%d'
end
-- Set the auto params
if dev_raw then
	print('dev_raw', dev_raw)
	for i, param in ipairs(metadata.auto_param) do
		local name, value = unpack(param)
		print('Setting Auto', name, value)
		--[[
		local before = camera:get_param(name)
		local ok = camera:set_param(name, value)
		local now = camera:get_param(name)
		if not ok then
			print(string.format('Failed %s: from %d to %d', name, before, value))
		end
		--]]
		local ext = string.format('uvcdynctrl -d %s -s %s %d', dev_raw, name, value)
		os.execute('uvcdynctrl -d'..dev_raw..' -s "Exposure, Auto" 1')
	end
end

-- Set the params
for i, param in ipairs(metadata.param) do
	local name, value = unpack(param)
	print('Setting', name, value)
	camera:set_param(name, value)
	--unix.usleep(1e5)
	local now = camera:get_param(name)
	-- TODO: exposure
	local count = 0
	while count<5 and now~=value do
		camera:set_param(name, value)
		now = camera:get_param(name)
		--unix.usleep(1e5)
		count = count + 1
	end
	if now~=value then
		print(string.format('Failed to set %s: %d -> %d',name, value, now))
	end
end

-- Cleanly exit on Ctrl-C
local running = true
local function shutdown()
  running = false
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

while running do
	local img, sz, cnt, t = camera:get_image()
	update(img, sz, cnt, t)
	if t-t_debug>1 then
		t_debug = t
		local kb = collectgarbage('count')
		local debug_str = {
			string.format("Cam %s | %ds, %d kB", name, t-t0, kb),
		}
		print(table.concat(debug_str,'\n'))
	end
	collectgarbage'step'
end
