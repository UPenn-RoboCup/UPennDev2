#!/usr/bin/env luajit
-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local Body = require'Body'
require'wcm'
require'mcm'

-- 2 Hz feedback
local t_sleep = 1e6 / 100
local feedback_udp_ch
local ret, err
local feedback = {}

local function entry()
	feedback_udp_ch =
	si.new_sender(Config.net.operator.wired, Config.net.streams.feedback.udp)
	print('Connected to', Config.net.operator.wired)
	print('Port', Config.net.streams.feedback.udp)
end

local function update()
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
	if err and Config.debug.feedback then
		print('Feedback UDP error',err)
	end
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=entry, update=update, exit=nil}
end

local running = true
local function shutdown()
	running = false
end
local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local usleep = require'unix'.usleep
local debug_interval = 2
local t0 = Body.get_time()
local t, t_debug = 0, 0
entry()
while running do
	update()
  t = Body.get_time()
    -- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
    local kb = collectgarbage('count')
    print(string.format('Feedback | Uptime: %d sec, Mem: %d kB', t-t0, kb))
  end

  collectgarbage('step')
	usleep(t_sleep)

end
