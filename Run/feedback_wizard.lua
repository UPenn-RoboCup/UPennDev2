#!/usr/bin/env luajit
-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local Body = require'Body'
local get_time = Body.get_time
local usleep = require'unix'.usleep
local debug_interval = 2
local feedback_interval = 1
local ping_rate = 5
local t_sleep = 1e6 / ping_rate
require'wcm'
require'mcm'

local feedback_udp_ch, ping_ch
local ret, err
local feedback = {}
local nBytes = 0

local function entry()
	feedback_udp_ch = si.new_sender(
		Config.net.operator.wired,
		Config.net.streams.feedback.udp
	)
	-- Lossy channel test
	ping_ch = si.new_sender(Config.net.operator.wired, Config.net.test)
end


local function update()
	ping_ch:send'chocolate'
	if t - t_feedback < feedback_interval then return end
	feedback.t = Body.get_time()
	feedback.p = Body.get_position()
	feedback.cp = Body.get_command_position()
	feedback.i = Body.get_current()
	feedback.rpy = Body.get_rpy()
	feedback.gyro = Body.get_gyro()
	feedback.pose = wcm.get_robot_odometry()--wcm.get_robot_pose()
	feedback.bh = mcm.get_stance_bodyHeight()
	feedback.battery = Body.get_battery()
	ret, err = feedback_udp_ch:send(mpack(feedback))
	if err then
		io.write('Feedback | UDP error: ', err)
	else
		nBytes = nBytes + ret
	end
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=entry, update=update, exit=nil}
end

local running = true
local function shutdown() running = false end
local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local t0 = get_time()
local t, t_debug = 0, 0
entry()
while running do
	update()
  t = get_time()
	-- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
    local kb = collectgarbage('count')
    io.write(string.format(
			'Feedback | Uptime: %d sec, Mem: %d kB, Sent: %d bytes',
			t-t0, kb, nBytes)
		)
  end
	-- Sleep a bit
  collectgarbage('step')
	usleep(t_sleep)
end
