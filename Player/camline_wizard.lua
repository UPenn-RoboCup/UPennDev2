-----------------------------------
-- Camera manager for finding lines
-- (c) Stephen McGill, 2014
-----------------------------------
-- Something there is non-reentrant
dofile'../include.lua'

-- Which camera
local metadata = Config.camera[1]
-- Who to send to
local operator = Config.net.operator.wired
local udp_port = Config.net.camera[name]

local torch = require'torch'
local ImageProc2 = require'ImageProc.ffi'
local mp = require'msgpack.MessagePack'
local si = require'simple_ipc'
local udp = require'udp'
local uvc = require'uvc'
local jpeg = require'jpeg'
local util = require'util'
local vector = require'vector'
local Body = require'Body'
local K = Body.Kinematics

-- Channels
local edge_ch = si.new_publisher('edge')
local camera_ch = si.new_publisher('camera0')
local tou_che = si.new_subscriber('touche')
local line_ch = si.new_publisher('line')
-- UDP Sending
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
local focal_length = metadata.focal_length

-- Garbage collection
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

-- Update the distance measurements!
local T = require'libTransform'
local last_measurement
local sin, cos = math.sin, math.cos
local function update_dist (pline1, pline2, tr)
	-- Find the distance between the lines
	local p0 = vector.new{pline2.iMin, pline2.jMin}
	local p1 = vector.new{pline1.iMean, pline1.jMean}
	local p2 = vector.new{pline2.iMax, pline2.jMax}
	local a = vector.norm(p1 - p0)
	local b = vector.norm(p2 - p0)
	local c = vector.norm(p2 - p1)
	local angle = math.acos( (a^2+b^2-c^2) / (2 * a * b) )
	local px_width = a * sin(angle)
	-- Get the angles
	local i_px1, i_px2 = pline1.iMean - (w / 2), pline2.iMean - (w / 2)
	local camera_angle1 = math.atan(i_px1 / focal_length)
	local camera_angle2 = math.atan(i_px2 / focal_length)
	local angle_width = (camera_angle2 + camera_angle1) / 2
	-- See if this is our first mesaurment
	if not last_measurement then
		last_measurement = {
			px_width = px_width,
			tr = tr,
			angle_width = angle_width,
		}
		return
	end
	-- If the less than a few pixels difference in width, then return
	if width - last_measurement.width < 5 then return end
	-- Check the distance between transforms
	local p_last, p_now = T.get_pos(last_measurement.tr), T.get_pos(tr)
	local p_diff = vector.norm(p_last - p_now)
	-- If less than an inch, discard
	if p_diff < 0.0254 then return end
	local sin_diff = sin(angle_width - last_measurement.angle_width)
	local s_a = sin(angle_width)
	local r = (s_a * sin(last_measurement.angle_width)) / sin_diff * p_diff
	local d = (s_a * cos(last_measurement.angle_width)) / sin_diff * p_diff - r
	-- Update the last_measurment
	last_measurement = {
		px_width = px_width,
		tr = tr,
		angle_width = angle_width,
	}
	-- Return the distance measurement
	return r, d
end

-- Variables
local kernel_t, use_horiz, use_vert = ImageProc2.dir_to_kernel(), true, true
-- Form the default bounding box (in scaled down space...)
local bbox = {51, 101, 21, 111}
ImageProc2.setup(w, h, 2, 2)

-- Updating stuff
local function update_bbox ()
	local bbox_data = tou_che:receive(true)
	if not bbox_data then return end
	-- Just use the first one...
	local bb = mp.unpack(bbox_data[1])
	bbox = vector.new(bb.bbox) / 2
	for i,v in ipairs(bbox) do bbox[i] = math.ceil(v) end
	local dir = bb.dir
	kernel_t = ImageProc2.dir_to_kernel(dir)
	if dir=='v' then
		use_horiz, use_vert = false, true
	elseif dir=='h' then
			use_horiz, use_vert = true, false
	else
			use_horiz, use_vert = true, true
	end
	print('BBOX', bbox, dir)
	util.ptorch(kernel_t)
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
	-- Send to the human user
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
		pline1.iMin  = 2 * (pline1.iMin  + bbox[1])
		pline1.iMean = 2 * (pline1.iMean + bbox[1])
		pline1.iMax  = 2 * (pline1.iMax  + bbox[1])
		pline1.jMin  = 2 * (pline1.jMin  + bbox[3])
		pline1.jMean = 2 * (pline1.jMean + bbox[3])
		pline1.jMax  = 2 * (pline1.jMax  + bbox[3])
		pline2.iMin  = 2 * (pline2.iMin  + bbox[1])
		pline2.iMean = 2 * (pline2.iMean + bbox[1])
		pline2.iMax  = 2 * (pline2.iMax  + bbox[1])
		pline2.jMin  = 2 * (pline2.jMin  + bbox[3])
		pline2.jMean = 2 * (pline2.jMean + bbox[3])
		pline2.jMax  = 2 * (pline2.jMax  + bbox[3])

		line_ch:send(mp.pack({
			name = 'pline',
			l1 = {
				{x=pline1.iMin,  y=pline1.jMin},
				{x=pline1.iMean, y=pline1.jMean},
				{x=pline1.iMax,  y=pline1.jMax},
				},
			l2 = {
				{x=pline2.iMin,  y=pline2.jMin},
				{x=pline2.iMean, y=pline2.jMean},
				{x=pline2.iMax,  y=pline2.jMax},
				},
			-- Relative placement
			bbox = bbox,
			}))

		-- Tell the arm where to go
		-- WristYaw is the camera roll...
		local camera_roll = line_radon.ith * line_radon.NTH
		-- Shortest rotation to the point
		camera_roll = camera_roll > (math.pi / 2) and (camera_roll - math.pi) or camera_roll
		-- Place iMean in the center of the frame horizontally
		-- Remember, we massaged plines to be in the original resolution
		local i_px = (pline1.iMean + pline2.iMean) / 2 - (w / 2)
		local camera_angle_i = math.atan(i_px / focal_length)
		local j_px = (pline1.jMean + pline2.jMean) / 2 - (h / 2)
		local camera_angle_j = math.atan(j_px / focal_length)
		-- Now must set the pitch properly...
		local q = Body.get_command_position()
		local fk = K.forward_arm(q)
		-- Rotate
		local desired_tr = fk * T.rotY(camera_angle_j)
		local iqArm = vector.new(K.inverse_arm(desired_tr, qArm, false))
		iqArm[1] = camera_angle_i
		iqArm[5] = camera_roll
		-- Send the commands
		Body.set_command_position(iqArm)

		-- Update the distance to the wire and the wire's radius
		update_dist(pline1, pline2)
	end

	-- Collect garbage every cycle
	collectgarbage()

end
