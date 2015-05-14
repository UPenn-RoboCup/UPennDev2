#!/usr/bin/env luajit
dofile'../include.lua'
local si = require'simple_ipc'
local util = require'util'
local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack

local function get_armplan(plan)
	local lco, rco = movearm.goto(unpack(plan))
	local lpath, rpath
	local wpath = {}
	if lco then
		lpath = {}
		while coroutine.status(lco)~='dead' do
			local okL, qLWaypoint, qWaistpoint = coroutine.resume(lco)
			table.insert(lpath, qLWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
		end
	end
	if rco then
		rpath = {}
		while coroutine.status(rco)~='dead' do
			local okR, qRWaypoint, qWaistpoint = coroutine.resume(rco)
			table.insert(rpath, qRWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
		end
	end
	-- TODO: Check that the waist was not twice populated
	return lpath, rpath, wpath
end

local poller, lut
local channels = {}
local function cb(skt)
	local ch_id = lut[skt]
	local ch = channels[ch_id]
	local mdata = skt:recv_all()
	local mdata, ranges = unpack(skt:recv_all())
	local meta = munpack(mdata)
	local result = ch.process(meta)
	if ch.kind=='rep' then
		-- if REQ/REP then reply
		local ret = ch:send(mp.pack(result))
	end
end

local plan_ch = si.new_replier('armplan')
plan_ch.callback = cb
plan_ch.process = get_armplan
table.insert(channels, plan_ch)

poller = si.wait_on_channels(channels)

local function shutdown()
  poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

lut = poller.lut
poller:start()
