#!/usr/bin/env luajit
----------------------------
-- Kinect2 manager
-- (c) Stephen McGill, 2014
----------------------------
dofile'../include.lua'
local c_rgb = require'jpeg'.compressor('rgb')
local freenects2 = require'freenect2'
get_time = unix.time

local ptable = require'util'.ptable
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local depth_ch = si.new_publisher'kinect2_depth'
local color_ch = si.new_publisher'kinect2_color'

local function entry()
  local serial_number, firmware_version = freenect2.init()
  print("serial_number, firmware_version:", serial_number, firmware_version)
end

local cnt = 0
local function update()
  local rgb, depth, ir = freenect2.update()
  local t = get_time()
  cnt = cnt + 1
  
  -- Send color
  local j_rgb = c_rgb:compress(rgb.data, rgb.width, rgb.height)
  rgb.data = nil
  rgb.sz = #j_rgb
  
  -- Send depth (TODO: zlib)
  local ranges = depth.data
  depth.data = nil
  depth.sz = #ranges
  
  if cnt % 5 == 0 then
    --print('SEND')
    --ptable(rgb)
    --ptable(depth)
    --color_ch:send({mp.pack(rgb), j_rgb})
    depth_ch:send({mp.pack(depth), ranges})
  end
  return t
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

local t0 = get_time()
local t_debug = 0

entry()
while running do
  local t = update()
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