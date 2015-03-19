#!/usr/bin/env luajit
dofile'../include.lua'
local signal = require'signal'.signal
local running = true

local function shutdown()
	running = false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local Body = require('Body')
local si = require'simple_ipc'
local libLog = require'libLog'
local logger = libLog.new'joint'
require'wcm'
local sample_hz = 120

local get_time, usleep, max = unix.time, unix.usleep, math.max
local t0, t_sleep = get_time(), 1 / sample_hz
local count, t, t_debug = 0, t0, t0
print('Begin logging joints...')
local e = {}
while running do
  count = count + 1
  t = get_time()
	-- Update the entry
	e.t = t
	e.n = count
	e.cp, e.t_cp = Body.get_command_position()
	e.p, e.t_p = Body.get_position()
	e.i = Body.get_current()
	e.ft_l = Body.get_lfoot()
	e.ft_r = Body.get_rfoot()
	e.gyro, e.t_imu = Body.get_gyro()
	e.acc = Body.get_accelerometer()
	e.rpy = Body.get_rpy()
	e.pose = wcm.get_robot_odometry()
	e.battery = Body.get_battery()
	-- Write the log
	logger:record(e)
	-- Status message
	if t - t_debug > 10 then
		t_debug = t
		io.write('Joint Logger: ', count,'\n')
		logger:stop()
		logger = libLog.new('joint', true)
		io.write('Open new log!\n')
	end
  -- Garbage collection for timing reasons
  collectgarbage('step')
  t_diff = get_time() - t
  usleep(1e6 * max(t_sleep - t_diff, 0))
end

logger:stop()
