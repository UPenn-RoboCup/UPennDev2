#!/usr/bin/env luajit
dofile'../include.lua'

local root = HOME..'/Data/'
local episode = 'corner_walk'
local LOG_DIR = root..episode

local logs = {
{'03.25.2015.16.36.47', 1, 20}
}

local LOG_DATE, EX0, EX1 = unpack(logs[1])

local libLog = require'libLog'
local replay_depth = libLog.open(LOG_DIR, LOG_DATE, 'k2_depth')
local replay_rgb = libLog.open(LOG_DIR, LOG_DATE, 'k2_rgb')
local metadata = replay_depth:unroll_meta()
local metadata_rgb = replay_rgb:unroll_meta()
print('Unlogging', #metadata, 'images from', LOG_DATE)

local util = require'util'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local depth_ch = si.new_publisher'kinect2_depth'
local color_ch = si.new_publisher'kinect2_color'

local logged_depth = replay_depth:log_iter()
local logged_rgb = replay_rgb:log_iter()

local get_time = unix.time
local metadata_t0 = metadata[1].t
local t0

EX0 = EX0 or 1
EX1 = EX1 or #metadata

for i, metadata_depth, payload_depth in logged_depth do
	local i_rgb, metadata_rgb, payload_rgb = logged_rgb()
  metadata_rgb.c = 'jpeg'

	io.write('Count ', i, '\n')

	local t = get_time()
	t0 = t0 or t
	local dt = t - t0

	local metadata_dt = metadata_depth.t - metadata_t0
	local t_sleep = metadata_dt - dt
	--if t_sleep>0 then unix.usleep(1e6*t_sleep) end
	if i>=EX0 then
		metadata_depth.width = metadata_depth.width
		metadata_depth.height = metadata_depth.height
		depth_ch:send({mp.pack(metadata_depth), payload_depth})
		color_ch:send({mp.pack(metadata_rgb), payload_rgb})
		unix.usleep(1e6)
	end
	if i>=EX1 then return end
end
