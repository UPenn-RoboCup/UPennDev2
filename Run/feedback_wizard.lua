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
local ping_rate = 4
local t_sleep = 1e6 / ping_rate
require'wcm'

local feedback_udp_ch, ping_ch
local ret, err
local nBytes, nBytesPing = 0, 0
local t = 0
local t_feedback = 0

local function entry()
	feedback_udp_ch = si.new_sender(
		Config.net.operator.wired,
		Config.net.streams.feedback.udp
	)
	-- Lossy channel test
	ping_ch = si.new_sender(Config.net.operator.wired, Config.net.test.udp)
end

local msg
local e = {}
local count = 0
local function update()
  msg = tostring(t)
	ret, err = ping_ch:send(msg)
	if type(ret)=='string' then
		io.write('Feedback | Ping error: ', ret, '\n')
	else
		nBytesPing = nBytesPing + #msg
	end
	if t - t_feedback < feedback_interval then return end

	count = count + 1
	e.t = t
	e.n = count
	e.b = Body.get_battery()
	e.cp, e.t_cp = Body.get_command_position()
	e.p, e.t_p = Body.get_position()
	e.i = Body.get_current()
	e.ft_l = Body.get_lfoot()
	e.ft_r = Body.get_rfoot()
	e.gyro, e.t_imu = Body.get_gyro()
	e.acc = Body.get_accelerometer()
	e.rpy = Body.get_rpy()
	e.odom = wcm.get_robot_odometry()

  --
  msg = mpack(e)
	ret, err = feedback_udp_ch:send(msg)
	if type(ret)=='string' then
		io.write('Feedback | UDP error: ', ret, '\n')
	else
		nBytes = nBytes + #msg
	end
  t_feedback = t
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
local t_debug = 0
entry()
while running do
	update()
  t = get_time()
	-- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
    local kb = collectgarbage('count')
    io.write(string.format(
			'Feedback | Uptime: %d sec, Mem: %d kB, Sent: %d bytes\nPing %d bytes\n',
			t-t0, kb, nBytes, nBytesPing)
		)
  end
	-- Sleep a bit
  collectgarbage('step')
	usleep(t_sleep)
end
