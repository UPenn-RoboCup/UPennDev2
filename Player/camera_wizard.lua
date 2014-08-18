#!/usr/bin/env luajit
-----------------------------------
-- Camera manager
-- (c) Stephen McGill, 2014
-----------------------------------
-- Something there is non-reentrant
dofile'../include.lua'
if type(arg)~='table' then IS_WEBOTS=true end
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local jpeg = require'jpeg'
local Body = require'Body'
require'hcm'
require'wcm'
local get_time = Body.get_time

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

local t_send, t_log = 0, 0
local LOG_INTERVAL = 1/5
local SEND_INTERVAL = .5

local ENABLE_NET
local ENABLE_LOG
if Config.enable_monitor then
  ENABLE_NET = true
end
if Config.enable_log then
  ENABLE_LOG = true
end

local libLog, logger

-- Extract metadata information
local w = metadata.w
local h = metadata.h
local name = metadata.name
-- Who to send to
local operator
if Config.net.use_wireless then
	operator = Config.net.operator.wireless
else
	operator = Config.net.operator.wired_broadcast
end
-- Network Channels/Streams
local camera_identifier = 'camera'..(camera_id-1)
local stream = Config.net.streams[camera_identifier]
local udp_ch = stream and stream.udp and si.new_sender(operator, stream.udp)
local camera_ch = stream and stream.sub and si.new_publisher(stream.sub)
print('Camera | ', operator, camera_identifier)

-- Metadata for the operator for compressed image data
local c_meta = {
	-- Required for rendering
	sz = 0,
	c = 'jpeg',
	-- Extra information
	t = 0,
	id = name..'_camera',
	n = 0,
}

-- Form the detection pipeline
local pipeline = {}
for _, d in ipairs(metadata.detection_pipeline) do
	local detect = require(d)
	-- Send which camera we are using
	detect.entry(metadata, Body)
	pipeline[d] = detect
end

-- JPEG Compressor
local c_grey = jpeg.compressor('gray')
local c_yuyv = jpeg.compressor('yuyv')
-- TODO: Control the downsampling mode
--c_yuyv:downsampling(2)
--c_yuyv:downsampling(1)

-- LOGGING
if ENABLE_LOG then
	libLog = require'libLog'
	-- Make the logger
	logger = libLog.new('yuyv', true)
end

local nlog = 0
local udp_ret, udp_err, udp_data
local t0 = get_time()
local t_debug = 0

local function update(img, sz, cnt, t)
	-- Update metadata
	c_meta.t = t
	c_meta.n = cnt

	-- Check if we are sending to the operator
	SEND_INTERVAL = 1 / hcm.get_monitor_fps()
	if ENABLE_NET and udp_ch and t-t_send > SEND_INTERVAL then
		local c_img = c_yuyv:compress(img, w, h)
		meta.sz = #c_img
		udp_data = mp.pack(c_meta)..c_img
		udp_ret, udp_err = udp_ch:send(udp_data)
	end

	-- Do the logging if we wish
	if ENABLE_LOG and t - t_log > LOG_INTERVAL then
		meta.rsz = sz
    meta.obs = wcm.get_obstacle_enable()
    meta.head = Body.get_head_command_position()
    meta.head = Body.get_head_position() --TODO: which one?
    meta.rpy = Body.get_rpy()
    meta.pose = wcm.get_robot_pose()
    --TODO: log joint angles
		for pname, p in pairs(pipeline) do meta[pname] = p.get_metadata() end
		logger:record(meta, img, sz)
		t_log = t
		nlog = nlog + 1
		if nlog % 10 == 0 then print("# camera logs: "..nlog) end
	end

	-- Update the vision routines
	for pname, p in pairs(pipeline) do
		p.update(img)
		if ENABLE_NET and udp_ch and p.send and t-t_send>SEND_INTERVAL then
			for _,v in ipairs(p.send()) do
				if v[2] then
					udp_data = mp.pack(v[1])..v[2]
				else
					udp_data = mp.pack(v[1])
				end
				udp_ret, udp_err = udp_ch:send(udp_data)
			end
			t_send = t
		end
	end
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=nil, update=update, exit=nil}
end


-- Open the camera
local camera = require'uvc'.init(metadata.dev, w, h, metadata.format, 1, metadata.fps)
-- Set the params
for i, param in ipairs(metadata.auto_param) do
	local name, value = unpack(param)
	camera:set_param(name, value)
	unix.usleep(1e5)
	local now = camera:get_param(name)
	assert(now==value, string.format('Failed to set %s: %d -> %d',name, value, now))
end
-- Set the params
for i, param in ipairs(metadata.param) do
	local name, value = unpack(param)
	camera:set_param(name, value)
	unix.usleep(1e5)
	local now = camera:get_param(name)
	-- TODO: exposure
	local count = 0
	while count<5 and now~=value do
		camera:set_param(name, value)
		unix.usleep(1e6)
		count = count + 1
		now = camera:get_param(name)
	end
	assert(now==value, string.format('Failed to set %s: %d -> %d',name, value, now))
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
			string.format("Camera | %s Uptime: %.2f Mem: %d kB", name, t-t0, kb),
			"# logs: "..nlog
		}
		print(table.concat(debug_str,'\n'))
	end
	collectgarbage'step'
end
