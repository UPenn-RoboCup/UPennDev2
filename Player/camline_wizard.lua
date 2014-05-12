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

local libLog = require'libLog'
local torch = require'torch'
local ImageProc2 = require'ImageProc.ffi'
local util = require'util'
local bit = require'bit'
local vector = require'vector'
local mp = require'msgpack.MessagePack'
local si = require'simple_ipc'
local udp  = require'udp'
local uvc  = require'uvc'
local jpeg = require'jpeg'

-- Channels
local edge_ch = si.new_publisher('edge')
local camera_ch = si.new_publisher('camera0')
local tou_che = si.new_subscriber('touche')
local line_ch = si.new_publisher('line')

-- LOGGING
local ENABLE_LOG = false
local libLog, logger, Body
if ENABLE_LOG then
	libLog = require'libLog'
	Body = require'Body'
	-- Make the logger
	logger = libLog.new('uvc',true)
end

-- Which camera
local metadata = Config.camera[1]

-- Extract metadata information
local w = metadata.width
local h = metadata.height
local fps = metadata.fps
local fmt = metadata.format
local name = metadata.name
local dev = metadata.dev
metadata = nil
-- Extract Config information
local operator = Config.net.operator.wired
local udp_port = Config.net.camera[name]
Config = nil

-- Debug
print(util.color('Begin','yellow'),name)

-- UDP Sending
local udp_ch = udp.new_sender(operator, udp_port)
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

-- Variables
local kernel_t, use_horiz, use_vert = ImageProc2.dir_to_kernel(), true, true
-- Form the default bounding box (in scaled down space...)
local bbox = {51, 101, 21, 111}
ImageProc2.setup(w, h, 2, 2)

-- Updating stuff
local function update_bbox ()
local bbox_data = tou_che:receive(true)
	if bbox_data then
		-- Just use the first one...
		local bb = mp.unpack(bbox_data[1])
		bbox = vector.new(bb.bbox) / 2
		for i,v in ipairs(bbox) do bbox[i] = math.ceil(v) end
		local dir = bb.dir
		print('BBOX', bbox, dir)
		kernel_t = ImageProc2.dir_to_kernel(dir)
		if dir=='v' then
			use_horiz, use_vert = false, true
		elseif dir=='h' then
			use_horiz, use_vert = true, false
		else
			use_horiz, use_vert = true, true
		end
		util.ptorch(kernel_t)
	end
end

-- Open the camera
local camera = uvc.init(dev, w, h, fmt, 1, fps)

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
local edge_t, grey_t = ImageProc2.yuyv_to_edge(img, bbox, false, kernel_t)
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
