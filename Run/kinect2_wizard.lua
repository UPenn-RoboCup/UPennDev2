#!/usr/bin/env luajit
----------------------------
-- Kinect2 manager
-- (c) Stephen McGill, 2014
----------------------------
dofile'../include.lua'
local ptable = require'util'.ptable
local mpack = require'msgpack.MessagePack'.pack
local depth_ch = require'simple_ipc'.new_publisher'kinect2_depth'
local color_ch = require'simple_ipc'.new_publisher'kinect2_color'
local c_rgb = require'jpeg'.compressor('rgb')

local cfg = Config.kinect
local Body = require'Body'
require'mcm'

--local has_detection, detection = pcall(require, cfg.detection)

local ENABLE_LOG = true
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
  if t - t_send > 1 then
    t_send = t
    local q, rpy, bh = Body.get_position(), Body.get_rpy(), mcm.get_walk_bodyHeight()
	  -- Send color
    rgb.t = t
		rgb.c = 'jpeg'
    rgb.q = q
    rgb.bh = bh
    rgb.rpy = rpy
    rgb.id = 'k2_rgb'
    local j_rgb = c_rgb:compress(rgb.data, rgb.width, rgb.height)
    rgb.data = nil
    rgb.sz = #j_rgb
    rgb.rsz = #j_rgb
    local m_rgb = mpack(rgb)
    color_ch:send({m_rgb, j_rgb})
	  -- Send depth (TODO: zlib)
    depth.c = 'raw'
    depth.q = q
    depth.bh = bh
    depth.rpy = rpy
    depth.id = 'k2_depth'
	  local ranges = depth.data
	  depth.data = nil
	  depth.sz = #ranges
    depth.rsz = #ranges
    local m_depth = mpack(depth)
    depth_ch:send({m_depth, ranges})
    -- Log
    if ENABLE_LOG then
      log_rgb:record(m_rgb, j_rgb)
      log_depth:record(m_depth, ranges)
    end
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
