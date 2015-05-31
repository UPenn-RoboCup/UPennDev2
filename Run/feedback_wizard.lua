#!/usr/bin/env luajit
-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local Body = require'Body'
local get_time = Body.get_time
local usleep = require'unix'.usleep
local pose_global = require'util'.pose_global
local debug_interval = 5


local TEMP_LIM = 70

local t_sleep = 1 / 20
local t_entry = get_time()
require'wcm'
require'mcm'
require'hcm'
local USE_ZLIB = true
local czlib
if USE_ZLIB then
	czlib = require'zlib.ffi'.compress
end

local hz_wrist_send = .5
local dt_wrist_send = 1/hz_wrist_send
local hz_head_send = 0.4
local dt_head_send = 1/hz_head_send
local hz_outdoor_send = 1
local dt_outdoor_send = 1/hz_outdoor_send

local pillar_ch = si.new_subscriber('pillars')
local ittybitty0_ch = si.new_subscriber(Config.net.streams.ittybitty0.sub)
local ittybitty1_ch = si.new_subscriber(Config.net.streams.ittybitty1.sub)
local pillars
local ittybitty0, ittybitty1

local feedback_udp_ch
local feedback_ch
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
	-- Assume open to start
	hcm.set_network_open(1)
	hcm.set_network_topen(t_entry)
	if IS_WEBOTS then
		feedback_ch = si.new_publisher(Config.net.streams.feedback.sub)
		--[[
		if IS_COMPETING then
			ping_ch = si.new_subscriber(Config.net.ping.sub)
			go_ch = si.new_publisher(Config.net.ping.pub)
		end
		--]]
	else
		feedback_udp_ch = si.new_sender(
		Config.net.operator.wired,
		Config.net.streams.feedback.udp
		)
		ittybitty0_udp_ch = si.new_sender(
		Config.net.operator.wired,
		Config.net.streams.ittybitty0.udp
		)
		ittybitty1_udp_ch = si.new_sender(
		Config.net.operator.wired,
		Config.net.streams.ittybitty1.udp
		)
		--[[
		if IS_COMPETING then
			ping_ch = si.new_subscriber(Config.net.ping.tcp, Config.net.operator.wired)
			go_ch = si.new_sender(Config.net.operator.wired, Config.net.ping.udp)
		end
		--]]
	end
end

local msg
local e = {}

local function update()
	local t_update = get_time()

	local p
	repeat
		msg = pillar_ch:receive(true)
		if msg then
			for i,v in ipairs(msg) do
				p = munpack(v)
			end
		end
	until not msg
	if p then pillars = p end

	local y0
	repeat
		msg = ittybitty0_ch:receive(true)
		if msg then
			y0 = msg[#msg] end
	until not msg
	if y0 then ittybitty0 = y0 end

	local y1
	repeat
		msg = ittybitty1_ch:receive(true)
		if msg then
			y1 = msg[#msg]
		end
	until not msg
	if y1 then ittybitty1 = y1 end

	-- send this only when maxxed out..,
	local qTemp = Body.get_temperature()
	for i, tm in ipairs(qTemp) do
		if tm>TEMP_LIM then
			e.tm = qTemp
			break
		end
	end

	-- Default feedback
	e.u = get_torso()
	e.p = Body.get_command_position(1, 29)
	e.s = pillars
	--e.g = Body.get_rgrip_command_torque()

	--[[
	e.gp = Body.get_rgrip_position()
	e.gt = Body.get_rgrip_temperature()
	--]]

	--[[
	e.p = Body.get_position()
	e.p = Body.get_position()
	e.id = 'fb'
	e.t = t
	e.n = count
	e.fL = Body.get_lfoot()
	e.fR = Body.get_rfoot()
	e.b = Body.get_battery()
	e.i = Body.get_current()
	e.cp, e.t_cp = Body.get_command_position()
	e.p, e.t_p = Body.get_position()
	e.gyro, e.t_imu = Body.get_gyro()
	e.acc = Body.get_accelerometer()
	e.rpy = Body.get_rpy()
	e.pose = wcm.get_robot_pose()
	--]]

	--[[
	print()
	local channel = 9600*dt_image_send
	print('initial channel bits', channel)
	local fbmsg = mpack(e)
	--print('fbmsg bits', #fbmsg*8)
	local fbmsgz = czlib(fbmsg)
	print('fbmsgz bits', #fbmsgz*8, #fbmsg*8)
	channel = channel - #fbmsgz*8

	print('ittybitty0 bits', ittybitty0 and #ittybitty0*8)
	print('ittybitty1 bits', ittybitty1 and #ittybitty1*8)
	local channel0 = channel - (ittybitty0 and #ittybitty0*8 or 0)
	local channel1 = channel - (ittybitty1 and #ittybitty1*8 or 0)
	print('final meta  channel bits', channel)
	print('final itty0 channel bits', channel0)
	print('final itty1 channel bits', channel1)
	--]]

	local fbmsg = mpack(e)

	if feedback_ch then feedback_ch:send(fbmsg) end

	local available_bits = 0
	local available_bits0 = 0
	local is_indoors = hcm.get_network_indoors()
	local ret, err
	if is_indoors==2 and ittybitty0_udp_ch then
		-- send the ittybitty0 (head)
		if t_update - t_feedback < dt_head_send then return end
		available_bits0 = 9600*dt_head_send
		local fbmsgz = czlib(fbmsg)
		ret, err = feedback_udp_ch:send(fbmsgz)
		available_bits = available_bits0 - 8*#fbmsgz
		if ittybitty0 then
			ret, err = ittybitty0_udp_ch:send(ittybitty0)
			available_bits = available_bits - 8*#ittybitty0
		else
			print('No ittybitty0 to send')
		end
	elseif is_indoors==3 and ittybitty1_udp_ch then
		-- send the ittybitty1 (wrist)
		if t_update - t_feedback < dt_wrist_send then return end
		available_bits0 = 9600*dt_wrist_send
		local fbmsgz = czlib(fbmsg)
		ret, err = feedback_udp_ch:send(fbmsgz)
		available_bits = available_bits0 - 8*#fbmsgz
		if ittybitty1 then
			ret, err = ittybitty1_udp_ch:send(ittybitty1)
			available_bits = available_bits - 8*#ittybitty1
		else
			print('No ittybitty1 to send')
		end
	elseif feedback_udp_ch then
		if t_update - t_feedback < dt_outdoor_send then return end
		available_bits0 = 9600*dt_outdoor_send
		local fbmsgz = czlib(fbmsg)
		ret, err = feedback_udp_ch:send(fbmsgz)
		available_bits = available_bits0 - 8*#fbmsgz
	else
		return
	end

	t_feedback = t_update
	print('available_bits', available_bits, '/', available_bits0)
	if type(available_bits)=='number' and available_bits < 0 then
		local twait = math.max( 0.05, math.min(math.abs(available_bits / 9600), 0.5))
		print('fb | wait!', twait)
		unix.usleep(math.floor(twait*1e6))
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
		'FB | %d sec, %d kB\n',
		t-t0, kb)
		)
	end
	-- Sleep a bit
	collectgarbage('step')
	usleep(t_sleep)
end
