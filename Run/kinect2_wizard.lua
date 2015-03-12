#!/usr/bin/env luajit
local ENABLE_LOG = true
----------------------------
-- Kinect2 manager
-- (c) Stephen McGill, 2014
----------------------------
dofile'../include.lua'
local cfg = Config.kinect
local Body = require'Body'
require'mcm'
local ptable = require'util'.ptable
local mpack = require'msgpack.MessagePack'.pack

local operator
if Config.net.use_wireless then
	operator = Config.net.operator.wireless
else
	operator = Config.net.operator.wired
end
print(Config.net.streams['kinect2_depth'].tcp, operator)
local depth_net_ch, color_net_ch
local depth_udp_ch, color_udp_ch
if Config.IS_COMPETING then
	depth_udp_ch = require'simple_ipc'.new_sender(operator, Config.net.streams['kinect2_depth'].udp)
	color_udp_ch = require'simple_ipc'.new_sender(Config.net.streams['kinect2_color'].udp, operator)
else
	depth_net_ch = require'simple_ipc'.new_publisher(Config.net.streams['kinect2_depth'].tcp)
	color_net_ch = require'simple_ipc'.new_publisher(Config.net.streams['kinect2_color'].tcp)
end

local depth_ch = require'simple_ipc'.new_publisher'kinect2_depth'
local color_ch = require'simple_ipc'.new_publisher'kinect2_color'
print(depth_net_ch.name)

local c_rgb
if IS_WEBOTS then c_rgb = require'jpeg'.compressor('rgb') end
local T = require'Transform'
local rotY = T.rotY
local rotZ = T.rotZ
local trans = T.trans
local from_rpy_trans = T.from_rpy_trans
local flatten = T.flatten

-- CoM to the Neck (32cm in z)
local tNeck = trans(unpack(Config.head.neckOffset))
-- Mounting of Kinect from the neck axes
--local tKinect = trans(unpack(cfg.mountOffset))
local tKinect = from_rpy_trans(unpack(cfg.mountOffset))

-- Next rotation
local function get_transform(head_angles, imu_rpy, body_height)
  -- {yaw, pitch}
  return from_rpy_trans({imu_rpy[1], imu_rpy[2], 0}, {0, 0, body_height}) * tNeck * rotZ(head_angles[1]) * rotY(head_angles[2]) * tKinect
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
	-- Send debug
	if Config.IS_COMPETING and t - vcm.get_network_tgood() > 1 then return t end
  if t - t_send < 0.1 then return t end

  t_send = t
	print('Kinect2 | t_send', t_send)
  local rpy = Body.get_rpy()
  local bh = mcm.get_walk_bodyHeight()
  local qHead = Body.get_head_position()
  local tr = flatten(get_transform(qHead, rpy, bh))
	local odom = mcm.get_status_odometry()
	local vel = mcm.get_walk_vel()
  -- Form color
  rgb.t = t
  rgb.c = 'jpeg'
  rgb.id = 'k2_rgb'
  rgb.head_angles = qHead
  rgb.body_height = bh
  rgb.imu_rpy = rpy
  rgb.tr = tr
	rgb.odom = odom
	rgb.vel = vel
  local j_rgb
  if IS_WEBOTS then
    j_rgb = c_rgb:compress(rgb.data, rgb.width, rgb.height)
  else
    j_rgb = rgb.data
  end
  rgb.data = nil
  rgb.sz = #j_rgb
  rgb.rsz = #j_rgb
  local m_rgb = mpack(rgb)

  -- Form depth (TODO: zlib)
  depth.t = t
  depth.c = 'raw'
  depth.id = 'k2_depth'
  depth.head_angles = qHead
  depth.body_height = bh
  depth.imu_rpy = rpy
  depth.tr = tr
	depth.odom = odom
	depth.vel = vel
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
	if Config.IS_COMPETING then
		color_udp_ch:send(m_rgb..j_rgb)
		depth_udp_ch:send(m_depth..ranges)
	end
	color_net_ch:send({m_rgb, j_rgb})
  depth_net_ch:send({m_depth, ranges})
	color_ch:send({m_rgb, j_rgb})
  depth_ch:send({m_depth, ranges})

  -- Log at 2Hz
	if t - t_send < 0.25 then return t end
  if ENABLE_LOG then
    log_rgb:record(m_rgb, j_rgb)
    log_depth:record(m_depth, ranges)
  end

  return t
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=has_detection and detection.entry, update=update, exit=has_detection and detection.exit}
end

local freenect2 = require'freenect2'

local function entry()
  local serial_number, firmware_version = freenect2.init()
  print("serial_number, firmware_version:", serial_number, firmware_version)
	if has_detection then detection.entry() end
end
local function exit()
  freenect2.shutdown()
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

entry()
while running do
	local rgb, depth, ir = freenect2.update()
	local t = get_time()
	rgb.t = t
	depth.t = t
	update(rgb, depth)
	if t-t_debug>1 then
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
