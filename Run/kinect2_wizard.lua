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

local cnt = 0
local function update(metadata, rgb, depth)
  cnt = cnt + 1
  if cnt % 5 == 0 then
	  -- Send color
	  local j_rgb = c_rgb:compress(rgb.data, rgb.width, rgb.height)
	  rgb.data = nil
	  rgb.sz = #j_rgb
		rgb.c = 'jpeg'
    color_ch:send({mpack(rgb), j_rgb})
	  -- Send depth (TODO: zlib)
	  local ranges = depth.data
	  depth.data = nil
	  depth.sz = #ranges
    depth_ch:send({mpack(depth), ranges})
  end
  return t
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=nil, update=update, exit=nil}
end

local freenects2 = require'freenect2'

local function entry()
  local serial_number, firmware_version = freenect2.init()
  print("serial_number, firmware_version:", serial_number, firmware_version)
end
local function exit()
  freenect2.shutdown()
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
	update({}, rgb, depth)
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