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
--local ENABLE_NET = false
local ENABLE_NET = true
--local FROM_LOG, LOG_DATE = true, '05.28.2014.16.18.44'
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
local udp_port = metadata.udp_port
local lA_port = metadata.lA_port


-- Channels
-- UDP Sending
--local camera_ch = si.new_publisher('camera0')
local udp_ch, lA_ch, detect_ch
operator = '192.168.123.200' -- TODO
if udp_port then udp_ch = udp.new_sender(operator, udp_port) end
if lA_port then lA_ch = udp.new_sender(operator, lA_port) end
detect_ch = udp.new_sender(operator, Config.net.detect)

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
	logger = libLog.new('uvc', true)
end

-- Open the camera
local camera = uvc.init(metadata.dev, w, h, metadata.format, 1, metadata.fps)
-- Set the params
for i, param in ipairs(metadata.auto_param) do
	local name, value = unpack(param)
	camera:set_param(name, value)
	unix.usleep(1e5)
	assert(camera:get_param(name)==value, 'Failed to set '..name)
end
for i, param in ipairs(metadata.param) do
	local name, value = unpack(param)
	camera:set_param(name, value)
	unix.usleep(1e5)
	print(name, value, camera:get_param(name))
	--TODO:C905 somehow won't work after re-plugh
	assert(camera:get_param(name)==value, 'Failed to set '..name)
end

require'vcm'
local ImageProc = require'ImageProc'
if jit then ImageProc2 = require'ImageProc.ffi' end
local HT = require'HeadTransform'
local detectBall = require'detectBall'
local detectGoal = require'detectGoal'
local World = require'World'

-- Define Color
local colorOrange = Config.vision.colorOrange
local colorYellow = Config.vision.colorYellow
local colorWhite = Config.vision.colorWhite


local labelA, labelB, scaleA, scaleB, lut
local ball, goal, line = {}, {}, {}
local function initialize()
  labelA,labelB = {},{}
  scaleA, scaleB = Config.vision.scaleA, Config.vision.scaleB
	print('NEW LABELING!!')

  ImageProc2.setup(w, h, scaleA, scaleB)
  -- Load colortable
  local lut_filename = HOME.."Data/"..Config.camera[1].lut
  local _, lut_id = ImageProc2.load_lut(lut_filename)
  lut = ImageProc2.get_lut(lut_id):data() -- TODO
  print('LOADED '..lut_filename)

	ball.detect, goal.detect, line.detect = 0,0,0
	vcm.set_ball_detect(0)
	vcm.set_goal_detect(0)
	vcm.set_goal_type(0)
	vcm.set_goal_enable(0)
	
	--World.entry()
end

local function send_overlay(ball, goal, line)
	local data={}
	data.wA, data.hA = labelA.m, labelA.n
	----- Ball
  data.ball = {}
  data.ball.ballX = wcm.get_ball_x()
  data.ball.ballY = wcm.get_ball_y()
  data.ball.ballDetect = ball.detect
  data.ball.debug_msg = ball.debug_msg
  if ball.detect>0 then
	  data.ball.ballCenter = vcm.get_ball_centroid()
	  data.ball.ballSize = vcm.get_ball_diameter()
	end
	----- Goal
	data.goal = {}
	data.goal.goalDetect = goal.detect
	data.goal.debug_msg = goal.debug_msg
	if goal.detect>0 then
		data.goal.goalType = goal.type -- means nothing
    data.goal.positions = goal.v
    data.goal.goalBBox = goal.goalBBox
    data.goal.centroids = {}
    data.goal.heights = {}
    data.goal.widths = {}
    for i=1,#goal.propsA do  -- TODO: might only need first item
	    data.goal.centroids[i] = goal.propsA[i].centroid
	    data.goal.heights[i] = goal.propsA[i].axisMajor
	    data.goal.widths[i] = goal.propsA[i].axisMinor
	  end
	end
	----- Line
	data.line = {}
  data.line.lineDetect = line.detect
  data.line.debug_msg = line.debug_msg
  if line.detect>0 then
	  data.line.endpointXY = line.vendpoint
	  data.line.endpointIJ = line.endpoint
	  data.line.nLines = line.nLines
	end

	--Send
	local ret, err = detect_ch:send(mp.pack(data))
	if err then print(ret, err) end
end


initialize()
while true do
	-- Grab and compress
	local img, sz, cnt, t = camera:get_image()
	-- Update metadata
	meta.t = t
	meta.n = cnt
	
	-- Generate label
	labelA_t = ImageProc2.yuyv_to_label(img, lut)
	labelB_t = ImageProc2.block_bitor(labelA_t)
	cc_t = ImageProc2.color_count(labelA_t)
	-- convert to light userdata
	--TODO: get rid of this, just use torch tensor
	local cutil = require'cutil'
	labelA.data = cutil.torch_to_userdata(labelA_t)
	colorCount  = cc_t
	labelB.data = cutil.torch_to_userdata(labelB_t)

	-- Label param
	labelA.m, labelA.n = w/scaleA, h/scaleA
	labelA.npixel = labelA.m * labelA.n
	labelB.m, labelB.n = labelA.m/scaleB, labelA.n/scaleB
	labelB.npixel = labelB.m * labelB.n

	-- Update head transform per vision update
	--TODO: is this still reading command position?
	local headAngles = Body.get_head_position()
	HT.update(headAngles, labelA, labelB, metadata.focal_length, metadata.focal_base)

	-- Detection
	ball = detectBall.detect(colorOrange,colorCount,labelA,labelB,HT,t)
	goal = detectGoal.detect(colorYellow,colorCount,labelA,labelB,HT,t)
	
	
	-- Check if we are sending to the operator
	if ENABLE_NET then
		if udp_ch then
			local c_img = c_yuyv:compress(img, w, h)
			meta.sz = #c_img
			local udp_ret, err = udp_ch:send( mp.pack(meta)..c_img )
			if err then print(err, udp_ret) end
		end

		if lA_ch then
			local wA, hA = labelA.m, labelA.n
			local labelA_img = ImageProc.label_to_yuyv(labelA.data,wA,hA)
			local c_img = c_yuyv:compress(labelA_img, wA, hA)
			meta.width, meta.height, meta.sz = wA, hA, #c_img
			local udp_ret, err = lA_ch:send(mp.pack(meta)..c_img)
			if err then print('label udp err', err) end
		end

		if detect_ch then
			send_overlay(ball, goal, line) -- TODO:world
		end
	end

	-- Do the logging if we wish
	if ENABLE_LOG then
		meta.rsz = sz
		for pname, p in pairs(pipeline) do meta[pname] = p.get_metadata() end
		logger:record(meta, img, sz)
	end

	if t-t_debug>1 then
		t_debug = t
		print("DEBUG",t)
	end

	-- Collect garbage every cycle
	collectgarbage()
end
