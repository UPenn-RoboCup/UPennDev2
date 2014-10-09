#!/usr/bin/env luajit
-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local Body = require'Body'

-- 2 Hz feedback
local t_sleep = 1e6 / 2
local feedback_udp_ch
local ret, err
local feedback = {}

local function entry()
  feedback_udp_ch =
    si.new_sender(Config.net.operator.wired, Config.net.streams.feedback.udp)
end

local function update()
  feedback.t = Body.get_time()
  feedback.joints = Body.get_position()
	feedback.pose = wcm.get_robot_odometry()--wcm.get_robot_pose()
	feedback.rpy = Body.get_rpy()
  feedback.gyro = Body.get_gyro()
  feedback.height = mcm.get_stance_bodyHeight()
  feedback.battery = Body.get_battery()
	ret, err = feedback_udp_ch:send(mpack(feedback))
	if err and Config.debug.feedback then print('Feedback UDP error',err) end
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
entry()
while running do
	update()
	usleep(t_sleep)
end
