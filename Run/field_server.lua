#!/usr/bin/env luajit
dofile'../include.lua'
local si = require'simple_ipc'
local util = require'util'
local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack

local function get_plane(plane)
	print('Received a plane')
	util.ptable(plane)
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

-- TODO: Add ports to the config
local plane_ch = si.new_subscriber(1999, Config.net.field_computer)
util.ptable(plane_ch)
plane_ch.callback = cb
plane_ch.process = get_plane
table.insert(channels, plane_ch)

poller = si.wait_on_channels(channels)

local function shutdown()
  poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

lut = poller.lut
poller:start()
