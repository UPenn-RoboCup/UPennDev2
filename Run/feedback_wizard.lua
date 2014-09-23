#!/usr/bin/env luajit
-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local Body = require'Body'

-- 2 Hz feedback
local t_sleep = 1e6 / 2
local feedback_udp_ch =
si.new_sender(Config.net.operator.wired, Config.net.streams.feedback.udp)

local ret, err
local feedback = {}
local function update()
  feedback.t = Body.get_time()
  feedback.joints = Body.get_position()
	feedback.pose = wcm.get_robot_odometry()--wcm.get_robot_pose()
	feedback.rpy = Body.get_rpy()
  feedback.height = mcm.get_stance_bodyHeight()
  feedback.battery = Body.get_battery()
	ret, err = feedback_udp_ch:send(mpack(feedback))
	if err then print('Feedback UDP error',err) end
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=nil, update=update, exit=nil}
end

local running = true
local function shutdown()
  running = false
end
local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local unix = require'unix'
while running do
	update()
	unix.usleep(t_sleep)
end
