#!/usr/bin/env luajit
-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local Body = require'Body'
local get_time = Body.get_time
local usleep = require'unix'.usleep
local pose_global = require'util'.pose_global
local debug_interval = 5
local feedback_interval = 1 / 2
local t_sleep = 1 / 20
local t_entry = get_time()
require'wcm'
require'mcm'
require'hcm'

local feedback_udp_ch
local feedback_ch
local ret, err
local nBytes, nBytesPing = 0, 0
local t_feedback = 0
local t_open = -math.huge

local function get_torso()
	local rpy = Body.get_rpy()
	local uComp = mcm.get_stance_uTorsoComp()
	uComp[3] = 0

	local torso0 = pose_global(uComp, mcm.get_status_bodyOffset())
	local pose = wcm.get_robot_pose()
	local torsoG = pose_global(torso0, pose)

	local bh = mcm.get_walk_bodyHeight()
	local qHead = Body.get_head_position()
	--local torso = {torso0.x, torso0.y, bh, rpy[1], rpy[2], torso0.a}
	local global = {torsoG.x, torsoG.y, bh, rpy[1], rpy[2], torsoG.a}
	return global
end

local function entry()
t_entry = get_time()
	if IS_WEBOTS then
		feedback_ch = si.new_publisher(Config.net.streams.feedback.sub)
		ping_ch = si.new_subscriber(Config.net.ping.sub)
		go_ch = si.new_publisher(Config.net.ping.pub)
	else
		feedback_ch = si.new_sender(
			Config.net.operator.wired,
			Config.net.streams.feedback.udp
		)
		ping_ch = si.new_subscriber(Config.net.ping.tcp, Config.net.operator.wired)
		go_ch = si.new_sender(
			Config.net.operator.wired,
			Config.net.ping.udp
		)
	end
end

local msg
local e = {}
local count = 0
local function update()
	local t_update = get_time()
	go_ch:send(mpack(t_update))
	local data = ping_ch:receive(true)
local is_open = hcm.get_network_open()==1
	if (not is_open) and data then
		hcm.set_network_open(1)
		hcm.set_network_topen(t_update)
		t_open = t_update
print('net open', t_open-t_entry)

	elseif is_open and t_update - t_open > 1 then
print('net closed', t_update-t_entry)
		hcm.set_network_open(0)
	end
	if not IS_WEBOTS and t_update - t_feedback < feedback_interval then return end

	count = count + 1
	e.id = 'fb'
	e.t = t
	e.n = count
	e.b = Body.get_battery()
	e.torso = get_torso()
	e.p = Body.get_position()
	e.ft_l = Body.get_lfoot()
	e.ft_r = Body.get_rfoot()
	--[[
	e.i = Body.get_current()
	e.cp, e.t_cp = Body.get_command_position()
	e.p, e.t_p = Body.get_position()
	e.gyro, e.t_imu = Body.get_gyro()
	e.acc = Body.get_accelerometer()
	e.rpy = Body.get_rpy()
	e.pose = wcm.get_robot_pose()
  --]]

  msg = mpack(e)
	if feedback_ch then feedback_ch:send(msg) end
	if feedback_udp_ch then ret, err = feedback_udp_ch:send(msg) end
	if type(ret)=='string' then
		io.write('Feedback | UDP error: ', ret, '\n')
	else
		nBytes = nBytes + #msg
	end
  t_feedback = t_update
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
	local t = get_time()
	-- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
    local kb = collectgarbage('count')
    io.write(string.format(
			'FB | %d sec, %d kB, %d bytes\n',
			t-t0, kb, nBytes, nBytesPing)
		)
  end
  -- Sleep a bit
  collectgarbage('step')
  usleep(t_sleep)
end
