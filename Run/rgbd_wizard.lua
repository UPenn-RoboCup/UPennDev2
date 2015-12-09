#!/usr/bin/env luajit
local ENABLE_LOG = false
----------------------------
-- Kinect2 manager
-- (c) Stephen McGill, 2014
----------------------------
dofile'../include.lua'
local cfg = Config.kinect
local Body = require'Body'
local util = require'util'
require'mcm'
require'wcm'
local ptable = require'util'.ptable
local mpack = require'msgpack.MessagePack'.pack
local jpeg = require'jpeg'
local c_rgb = jpeg.compressor('rgb')

local operator
if Config.net.use_wireless then
	operator = Config.net.operator.wireless
else
	operator = Config.net.operator.wired
end

local si = require'simple_ipc'

local depth_streams = Config.net.streams['kinect_depth']
local color_streams = Config.net.streams['kinect_color']

local depth_udp_ch = si.new_sender(operator, depth_streams.udp)
local color_udp_ch = si.new_sender(operator, color_streams.udp)
--
local depth_net_ch = si.new_publisher(depth_streams.tcp)
local color_net_ch = si.new_publisher(color_streams.tcp)
--
local depth_ch = si.new_publisher(depth_streams.sub)
local color_ch = si.new_publisher(color_streams.sub)


local c_rgb
if IS_WEBOTS then c_rgb = require'jpeg'.compressor('rgb') end
local T = require'Transform'
local transform6D = require'Transform'.transform6D
local rotY = T.rotY
local rotZ = T.rotZ
local trans = T.trans
local from_rpy_trans = T.from_rpy_trans
local flatten = T.flatten


-- CoM to the Neck (32cm in z)
local tNeck = trans(unpack(Config.head.neckOffset))
-- Mounting of Kinect from the neck axes
local tKinect = from_rpy_trans(unpack(cfg.mountOffset))

local function get_tf()
	local rpy = Body.get_rpy()
	local pose = wcm.get_robot_pose()
	local bh = mcm.get_walk_bodyHeight()
	local bo = mcm.get_status_bodyOffset()
	local qHead = Body.get_head_position()
	local uComp = mcm.get_stance_uTorsoComp()
	uComp[3] = 0

	-- Poses with the compensation
	local torsoL = util.pose_global(uComp, bo)
	local torsoG = util.pose_global(torsoL, pose)

	-- Transform relative to the local body frame (on the ground, between the feet)
	local tfTorsoLocal = transform6D{torsoL.x, torsoL.y, bh, rpy[1], rpy[2], torsoL.a}
	-- Transform relative to the global body frame (on the ground)
	local tfTorsoGlobal = transform6D{torsoG.x, torsoG.y, bh, rpy[1], rpy[2], torsoG.a}
	-- Transform relative to the center of mass
	local tfCom = tNeck * rotZ(qHead[1]) * rotY(qHead[2]) * tKinect

	return tfTorsoLocal * tfCom, tfTorsoGlobal * tfCom
end

--local has_detection, detection = pcall(require, cfg.detection)

local libLog, logger
if ENABLE_LOG then
	libLog = require'libLog'
	log_rgb = libLog.new('k2_rgb', true)
	log_depth = libLog.new('k2_depth', true)
end

local get_time = Body.get_time
local cnt, t, t_send = 0, 0, -2
local function update(rgb, depth)
	t = get_time()
	cnt = cnt + 1
	-- Process and send
	if has_detection then
		detection.update(rgb, depth)
		for _,v in ipairs(detection.send()) do color_ch:send({mp.pack(v[1]), v[2]}) end
	end
	-- Timing
	if IS_COMPETING and t - hcm.get_network_topen() > 1 then
		return t
	end
	if t - t_send < 1 then return t end
	t_send = t
	local tfL, tfG = get_tf()
	local tfL_flat, tfG_flat = flatten(tfL), flatten(tfG)
	local head_angles = Body.get_head_position()

	-- Form color
	rgb.t = t
	rgb.id = 'k_rgb'
	rgb.c = 'jpeg'
	rgb.tfL16 = tfL_flat
	rgb.tfG16 = tfG_flat
	rgb.head_angles = head_angles

	--local j_rgb = rgb.data
	local j_rgb = c_rgb:compress(rgb.data, rgb.width, rgb.height)
	rgb.data = nil
	rgb.sz = #j_rgb
	rgb.rsz = #j_rgb
	local m_rgb = mpack(rgb)

	-- Form depth (TODO: zlib)
	depth.t = t
	depth.id = 'k_depth'
	depth.c = 'raw'
	depth.tfL16 = tfL_flat
	depth.tfG16 = tfG_flat
	depth.head_angles = head_angles

	local ranges = depth.data
	depth.data = nil
	depth.sz = #ranges
	depth.rsz = #ranges
	local m_depth = mpack(depth)

	-- Range compression method
	--[[
	local ddata = depth:data()
	ffi.copy(ddata, payload_depth)
	local bdata = depth_byte:data()
	for ii=0,npx-1 do
	bdata[ii] = min(max((ddata[ii] - near)/(far-near), 0), 255)
	end
	local c_depth = p_compress(bdata)
	--]]

	-- Send
	if not IS_WEBOTS then io.write('Kinect | t_send ', t_send,'\n') end

	if depth_udp_ch then depth_udp_ch:send(m_depth..ranges) end
	if color_udp_ch then color_udp_ch:send(m_rgb..j_rgb) end

	depth_net_ch:send({m_depth, ranges})
	color_net_ch:send({m_rgb, j_rgb})
	depth_ch:send({m_depth, ranges})
	color_ch:send({m_rgb, j_rgb})

	-- Log at 4Hz
	--	if t - t_send < 0.25 then return t end
	if ENABLE_LOG then
		log_rgb:record(m_rgb, j_rgb)
		log_depth:record(m_depth, ranges)
		if log_rgb.n >= 10 then
			log_rgb:stop()
			log_depth:stop()
			print('Open new log!')
			log_rgb = libLog.new('k2_rgb', true)
			log_depth = libLog.new('k2_depth', true)
		end
	end

	return t
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=has_detection and detection.entry, update=update, exit=has_detection and detection.exit}
end

local openni = require'openni'
local depth, color

local function entry()
  openni.startup()
  depth, color = openni.stream_info()
  print('Depth')
  ptable(depth)
  print('Color')
  ptable(color)
	if has_detection then detection.entry() end
end
local function exit()
	openni.shutdown()
	if has_detection then detection.exit() end
	if ENABLE_LOG then
		log_rgb:stop()
		log_depth:stop()
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

local get_time = unix.time
local t0 = get_time()
local t_debug = 0

--print('Before entry()')
entry()
--print('After entry()')
while running do
	print('update...')
	local d, c = openni.update_rgbd()
	print('done')
	local t = get_time()
	color.t = t
	depth.t = t
  color.data = c
	depth.data = d
	--update(color, depth)
	if t-t_debug > 1 then
		t_debug = t
		local kb = collectgarbage('count')
		local debug_str = {
			string.format("Kinect2 | Uptime: %.2f Mem: %d kB", t-t0, kb)
		}
		print(table.concat(debug_str,'\n'))
	end
	collectgarbage'step'
end
exit()
