-----------------------------------
-- Camera manager for finding lines
-- (c) Stephen McGill, 2014
-----------------------------------
-- Something there is non-reentrant
dofile'../include.lua'
local metadata
if not arg or type(arg[1])~='string' then
	-- TODO: Find the next available camera
	metadata = Config.camera[1]
else
	local cam_id = arg[1]
	if tonumber(cam_id) then
		metadata = assert(Config.camera[tonumber(cam_id)], 'Bad # ID')
	else
		for _, c in ipairs(Config.camera) do
			if c.name ==cam_id then
				metadata = c
				break
			end
		end
		assert(metadata, 'Bad camera name')
	end
end

local ENABLE_NET = true
local ENABLE_LOG, LOG_INTERVAL, t_log = false, 1 / 5, 0
local FROM_LOG, LOG_DATE = false, '05.28.2014.16.18.44'
local libLog, logger

local udp = require'udp'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local jpeg = require'jpeg'

-- Extract metadata information
local w = metadata.w
local h = metadata.h
local name = metadata.name
-- Who to send to
local operator = Config.net.operator.wired

-- Form the detection pipeline
local pipeline = {}
for _, d in ipairs(metadata.detection_pipeline) do
	local detect = require(d)
	-- Send which camera we are using
	detect.entry(metadata, Body)
	pipeline[d] = detect
end

-- Channels
-- UDP Sending
--local camera_ch = si.new_publisher('camera0')
local udp_ch = metadata.udp_port and udp.new_sender(operator, metadata.udp_port)
print('UDP',operator, metadata.udp_port)

-- Metadata for the operator
local meta = {
	t = 0,
	n = 0,
	sz = 0,
	w = w,
	h = h,
	name = name..'_camera',
	c = 'jpeg',
}

-- JPEG Compressor
local c_yuyv = jpeg.compressor('yuyv')
local c_grey = jpeg.compressor('gray')

-- Garbage collection before starting
collectgarbage()
local t_debug = unix.time()

if FROM_LOG then

	local libLog = require'libLog'
	local replay = libLog.open(HOME..'/Logs/', LOG_DATE, 'uvc')
	local metadata = replay:unroll_meta()
	local util = require'util'
	print('Unlogging', #metadata, 'images from', LOG_DATE)
	local logged_data = replay:log_iter()
	for i, m, yuyv_t in logged_data do
		assert(m.w==w, 'Bad width')
		assert(m.h==h, 'Bad height')
		util.ptable(m)
		local t = unix.time()
		-- Check if we are sending to the operator
		if ENABLE_NET then
			local c_img = c_yuyv:compress(yuyv_t, w, h)
			meta.sz = #c_img
			local udp_ret, err = udp_ch:send( mp.pack(meta)..c_img )
		end
		-- Update the vision routines
		for pname, p in pairs(pipeline) do
			p.set_metadata(m[pname])
			p.update(yuyv_t:data())
		end
		-- Debugging
		if t-t_debug>1 then
			t_debug = t
			print("DEBUG")
		end
		-- Collect garbage every cycle
		collectgarbage()
		-- Sleep a little
		unix.usleep(1e6/30)
	end
	-- Finish
	os.exit()
end

local uvc = require'uvc'
-- LOGGING
if ENABLE_LOG then
	libLog = require'libLog'
	-- Make the logger
	logger = libLog.new('yuyv', true)
end

-- Open the camera
local camera = uvc.init(metadata.dev, w, h, metadata.format, 1, metadata.fps)
-- Set the params
for i, param in ipairs(metadata.auto_param) do
	local name, value = unpack(param)
	camera:set_param(name, value)
	unix.usleep(1e4)
	local now = camera:get_param(name)
	assert(now==value, string.format('Failed to set %s: %d -> %d',name, value, now))
end
-- Set the params
for i, param in ipairs(metadata.param) do
	local name, value = unpack(param)
	camera:set_param(name, value)
	unix.usleep(1e4)
	local now = camera:get_param(name)
	assert(now==value, string.format('Failed to set %s: %d -> %d',name, value, now))
end

while true do
	-- Grab and compress
	local img, sz, cnt, t = camera:get_image()
	-- Update metadata
	meta.t = t
	meta.n = cnt

	-- Check if we are sending to the operator
	if ENABLE_NET then
		local c_img = c_yuyv:compress(img, w, h)
		meta.sz = #c_img
		local udp_ret, err = udp_ch:send( mp.pack(meta)..c_img )
	end

	-- Do the logging if we wish
	if ENABLE_LOG and t - t_log > LOG_INTERVAL then
		meta.rsz = sz
		for pname, p in pairs(pipeline) do meta[pname] = p.get_metadata() end
		logger:record(meta, img, sz)
		t_log = t
	end

	-- Update the vision routines
	for pname, p in pairs(pipeline) do
		p.update(img)
		if ENABLE_NET and p.send then
			for _,v in ipairs(p.send()) do
				if v[2] then
					udp_ch:send(mp.pack(v[1])..v[2])
				else
					udp_ch:send(mp.pack(v[1]))
				end
			end
		end
	end

	if t-t_debug>1 then
		t_debug = t
		print("DEBUG",t)
	end

	-- Collect garbage every cycle
	collectgarbage('step')
end
