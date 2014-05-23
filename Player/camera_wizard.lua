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
-- If we wish to log
-- TODO: arg or in config?
local ENABLE_LOG = false
local ENABLE_NET = false

local uvc = require'uvc'
local udp = require'udp'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local jpeg = require'jpeg'

-- Extract metadata information
local w = metadata.width
local h = metadata.height
local name = metadata.name
-- Who to send to
local operator = Config.net.operator.wired
local udp_port = metadata.unreliable

-- Form the detection pipeline
local pipeline = {}
for _, d in ipairs(metadata.detection_pipeline) do
	local detect = require(d)
	-- Send which camera we are using
	detect.entry(metadata)
	table.insert(pipeline, detect)
end

-- Open the camera
local camera = uvc.init(metadata.dev, w, h, metadata.format, 1, metadata.fps)
-- Set the params
for k, v in pairs(metadata.param) do
	camera:set_param(k, v)
	unix.usleep(1e4)
	assert(camera:get_param(k)==v, 'Failed to set '..k)
end

-- Channels
-- UDP Sending
--local camera_ch = si.new_publisher('camera0')
local udp_ch
if udp_port then udp_ch = udp.new_sender(operator, udp_port) end

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

-- LOGGING
local libLog, logger
if ENABLE_LOG then
	libLog = require'libLog'
	-- Make the logger
	logger = libLog.new('uvc',true)
end

-- JPEG Compressor
local c_yuyv = jpeg.compressor('yuyv')
local c_grey = jpeg.compressor('gray')

-- Garbage collection before starting
metadata = nil
Config = nil
collectgarbage()

while true do
	-- Grab and compress
	local img, sz, cnt, t = camera:get_image()
	-- Update metadata
	meta.t = t
	meta.n = cnt
	-- Do the logging if we wish
	if ENABLE_LOG then
		meta.rsz = sz
		logger:record(meta, img, sz)
	end

	-- Check if we are sending to the operator
	if ENABLE_NET then
		local c_img = c_yuyv:compress(img, w, h)
		meta.sz = #c_img
		local udp_ret, err = udp_ch:send( mp.pack(meta)..c_img )
	end

	-- Update the vision routines
	for _, p in ipairs(pipeline) do p.update(img) end

	-- Collect garbage every cycle
	collectgarbage()
end
