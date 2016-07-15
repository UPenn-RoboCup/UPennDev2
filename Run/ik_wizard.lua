#!/usr/bin/env luajit
dofile'../include.lua'
local si = require'simple_ipc'
local util = require'util'
local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack
local movearm = require'movearm'
local T = require'Transform'

grasping = {}

table.insert(grasping, {
	right = false,
	left = {
		timeout=10,
		via='jacobian_preplan',
		--tr={0.55, 0.246, 0.0, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		--tr = {0.2739,    0.2757,    0.0597,   -0.5689,   -0.0543,   -0.0072},
		tr = {0.3238 ,   0.2753 ,   0.0624 ,  -0.5689 ,  -0.0543 ,  -0.0072},
		--qArmGuess = vector.new{-15, 60, 90, -120, -80, -70, 0}*DEG_TO_RAD,
		weights = {1,1,1},
	}	
})



	sequence = {unpack(grasping)}

	s = 1
	stage = sequence[s]
	lco, rco = movearm.goto(stage.left, stage.right)



	print('Received a plan')
	local lco, rco = movearm.goto(stage.left, stage.right)
	local wpath = {}
	local g_qLWaypoint={}
	local g_qRwaypoint={}
	--
	local lpath = {}
	if type(lco)=='thread' then
		while coroutine.status(lco)~='dead' do
			local okL, qLWaypoint, qWaistpoint = coroutine.resume(lco)
			print('Left', okL, qLWaypoint)
			g_qLWaypoint = qLWaypoint
			table.insert(lpath, qLWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
		end
	else
--		print('lco', lco)
	end
	local rpath = {}
	if type(rco)=='thread' then
		while coroutine.status(rco)~='dead' do
			local okR, qRWaypoint, qWaistpoint = coroutine.resume(rco)
			print('Right', okR, qRWaypoint)
			g_qRWaypoint = qRWaypoint

			table.insert(rpath, qRWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
		end
	else
--		print('rco', rco)
	end
	-- TODO: Check that the waist was not twice populated
--	print('Sending the paths',#lpath, #rpath, #wpath)
--	return {lpath, rpath, wpath}

	print('Final Left', lpath[table.getn(lpath)])
	print('Final Left', g_qLWaypoint)


















--[[
local function get_config(path)
	print(unpack(path))
	local data = Config
	for i, v in ipairs(path) do
		data = data[v]
		if not data then return end
	end
	return data
end

local function get_armplan(plan)
	print('Received a plan')
	local lco, rco = movearm.goto(plan.left, plan.right)
	local wpath = {}
	--
	local lpath = {}
	if type(lco)=='thread' then
		while coroutine.status(lco)~='dead' do
			local okL, qLWaypoint, qWaistpoint = coroutine.resume(lco)
			table.insert(lpath, qLWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
		end
	else
		print('lco', lco)
	end
	local rpath = {}
	if type(rco)=='thread' then
		while coroutine.status(rco)~='dead' do
			local okR, qRWaypoint, qWaistpoint = coroutine.resume(rco)
			table.insert(rpath, qRWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
		end
	else
		print('rco', rco)
	end
	-- TODO: Check that the waist was not twice populated
	print('Sending the paths',#lpath, #rpath, #wpath)
	return {lpath, rpath, wpath}
end

local poller, lut
local channels = {}
local function cb(skt)
	local ch_id = lut[skt]
	local ch = channels[ch_id]
	local mdata = unpack(skt:recv_all())
	local meta = munpack(mdata)
	local result = ch.process(meta)
	if ch.kind=='rep' then
		-- if REQ/REP then reply
		local ret = ch:send(mpack(result))
	end
end

local plan_ch = si.new_replier('armplan')
plan_ch.callback = cb
plan_ch.process = get_armplan
table.insert(channels, plan_ch)

local config_ch = si.new_replier('config')
config_ch.callback = cb
config_ch.process = get_config
table.insert(channels, config_ch)

poller = si.wait_on_channels(channels)

local function shutdown()
  poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

lut = poller.lut
poller:start()
]]--