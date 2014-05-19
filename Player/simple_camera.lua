-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------
-- TODO: Critical section for include
-- Something there is non-reentrant
dofile'../include.lua'

-- Config information
local operator = Config.net.operator.wired
local udp_port = Config.net.camera[name]
local metadata = Config.camera[1]

local util = require'util'
local mp = require'msgpack.MessagePack'
local si = require'simple_ipc'
local udp  = require'udp'
local uvc  = require'uvc'
local jpeg = require'jpeg'

-- Channels
local camera_ch = si.new_publisher'camera0'
local udp_ch = udp.new_sender(operator, udp_port)

-- LOGGING
local ENABLE_LOG = false
local libLog, logger, Body
if ENABLE_LOG then
	libLog = require'libLog'
	Body = require'Body'
	-- Make the logger
	logger = libLog.new('uvc',true)
end


-- Extract metadata information
local w = metadata.width
local h = metadata.height
local fps = metadata.fps
local fmt = metadata.format
local name = metadata.name
local dev = metadata.dev

-- For Garbage collection
metadata = nil
Config = nil

-- Metadata for the operator
local meta = {
	t = 0,
	n = 0,
	sz = 0,
	w = w,
	h = h,
	name = name..'_camera',
	c = 'jpeg',
	--arm = Body.get_command_position(),
}
-- JPEG Compressor
local c_yuyv = jpeg.compressor('yuyv')

-- Open the camera
local camera = uvc.init(dev, w, h, fmt, 1, fps)
local camera_fd = camera:descriptor()

-- Set up the polling
si.wait_on_channels({
	camera_fd
})

while true do
	-- Grab and compress
	local img, sz, cnt, t = camera:get_image()
	local c_img = c_yuyv:compress( img, w, h)
	-- Update metadata
	meta.t = t
	meta.n = cnt
	meta.sz = #c_img
	--meta.arm = Body.get_command_position()
	-- Send
	local udp_ret, err = udp_ch:send( mp.pack(meta)..c_img )
	--print('udp img',img,sz,cnt,t,udp_ret)
	if err then print(name,'udp error',err) end
	if ENABLE_LOG then
		meta.rsz = sz
		logger:record(meta, img, sz)
	end

t0 = unix.time()
-- Process line stuff
update_bbox()
local edge_t, grey_t = ImageProc2.yuyv_to_edge(img, bbox, true, kernel_t)
local RT = ImageProc2.radon_lines(edge_t, use_horiz, use_vert)
local pline1, pline2, line_radon = RT.get_parallel_lines()
t1 = unix.time()
--print("time", t1-t0, 1/(t1-t0))
if pline1 then
-- massage
pline1.iMin = pline1.iMin + bbox[1]
pline1.iMean = pline1.iMean + bbox[1]
pline1.iMax = pline1.iMax + bbox[1]
pline1.jMin = pline1.jMin + bbox[3]
pline1.jMean = pline1.jMean + bbox[3]
pline1.jMax = pline1.jMax + bbox[3]

pline2.iMin = pline2.iMin + bbox[1]
pline2.iMean = pline2.iMean + bbox[1]
pline2.iMax = pline2.iMax + bbox[1]
pline2.jMin = pline2.jMin + bbox[3]
pline2.jMean = pline2.jMean + bbox[3]
pline2.jMax = pline2.jMax + bbox[3]

	line_ch:send(mp.pack({
		name = 'pline',
		l1 = {
			{x=2*pline1.iMin,  y=2*pline1.jMin},
			{x=2*pline1.iMean, y=2*pline1.jMean},
			{x=2*pline1.iMax,  y=2*pline1.jMax},
		},
		l2 = {
			{x=2*pline2.iMin,  y=2*pline2.jMin},
			{x=2*pline2.iMean, y=2*pline2.jMean},
			{x=2*pline2.iMax,  y=2*pline2.jMax},
		},
		-- Relative placement
		bbox = bbox,
	}))
end

  -- Collect garbage every cycle
  collectgarbage()

end
