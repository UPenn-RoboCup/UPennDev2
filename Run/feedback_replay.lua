#!/usr/bin/env luajit
-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'

local LOG_DATE = '10.24.2014.14.12.36'

local si = require'simple_ipc'
local feedback_udp_ch =
si.new_sender(Config.net.operator.wired, Config.net.streams.feedback.udp)
local feedback_ch = si.new_publisher(Config.net.streams.feedback.sub)

local mpack = require'msgpack.MessagePack'.pack
local libLog = require'libLog'
local replay_joint = libLog.open(HOME..'/Data/', LOG_DATE, 'joint')
local metadata = replay_joint:unroll_meta()
print('Unlogging', #metadata, 'feedback points from', LOG_DATE)

local logged_joint = replay_joint:log_iter()

local get_time = unix.time
local usleep = unix.usleep
local metadata_t0 = metadata[1].t
local t0, meta_t
local feedback = {}

local function shutdown()
	print('Stopped at t=', meta_t)
	os.exit()
end
local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local min_cnt, max_cnt = 400, 600

for i, metadata_joint in logged_joint do

	local t = get_time()
	t0 = t0 or t
	local dt = t - t0
	meta_t = metadata_joint.t
	local metadata_dt = meta_t - metadata_t0

	if i>=min_cnt and i<=max_cnt then
		if i%10==0 then
			print('Count', i)
			collectgarbage('step')
		end
		t_offset = t_offset or metadata_dt
		local t_sleep = metadata_dt - dt - t_offset
		if t_sleep>0 then usleep(1e6 * t_sleep) end
		feedback_ch:send(mpack(metadata_joint))
	end
  
end
