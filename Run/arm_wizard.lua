#!/usr/bin/env luajit
dofile'../include.lua'
local si = require'simple_ipc'
local util = require'util'
local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack
local movearm = require'movearm'
local T = require'Transform'

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

	-- Optimize the paths
	local costs = {}
	local n = 0
	--n = 2
	n = 10
	if type(lpath)=='table' then
		for i=1,n do
			local lpath1, d = movearm.optimize(lpath)
			if lpath1 then
				lpath = lpath1
				costs[i] = d
			end
		end
	else
		print('No path!')
	end

	local cdiff = {}
	for i,c1 in ipairs(costs[1]) do
		cdiff[i] = costs[#costs][i]^2 - c1^2

	end

	local dsum = 0
	io.write('\n')
	for i,v in ipairs(cdiff) do
		io.write(string.format('%.3f ', v))
		dsum = dsum + v
	end
	io.write('\n',dsum,'\n')

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
