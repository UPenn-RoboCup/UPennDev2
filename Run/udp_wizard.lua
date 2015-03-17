#!/usr/bin/env luajit
dofile'../include.lua'
local Config = require'Config'
local si = require'simple_ipc'
local util = require'util'

if HOSTNAME=='alvin' or HOSTNAME=='teddy' then
	
	local s = si.new_subscriber(Config.net.test.tcp, Config.net.operator.wired)
	s.callback = function(skt)
		local data = skt:recv_all()
		for _, tping in ipairs(data) do vcm.set_network_tgood(tping) end
	end
	poller = si.wait_on_channels{s}
	lut = poller.lut
	poller:start()
	
	-- Don't forward if a robot
	os.exit()
end

local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack
local function procMP(data)
	local tbl, offset = munpack(data)
	--print('offset', offset)
	if offset==#data then
		return mpack(tbl)
	else
		return {mpack(tbl), data:sub(offset+1)}
	end
	
end

local function procRaw(data)
	--print('data',data, #data)
	return data
end


local nsz = 0
local poller, lut
local in_channels = {}
local out_channels = {}
local ch_processing = {}
local function cb(skt)
	local ch_id = lut[skt]
	local in_ch = in_channels[ch_id]
	local out_ch = out_channels[ch_id]
	
	local fn = ch_processing[ch_id]
	local sz = in_ch:size()
	local data
	while sz > 0 do
		nsz = nsz + sz
		data = in_ch:receive()
		out_ch:send(fn(data))
		sz = in_ch:size()
	end
end

for key,stream in pairs(Config.net.streams) do
	if type(stream.udp)=='number' then
		io.write('Forwarding ', key, ': ', stream.udp, ' -> ', stream.sub, '\n')
		local r = si.new_receiver(stream.udp)
		r.callback = cb
		table.insert(in_channels, r)
		local s = si.new_publisher(stream.sub)
		table.insert(out_channels, s)
		table.insert(ch_processing, procMP)
	end
end

-- Forward the ping test packets back to the robot
do
	local r = si.new_receiver(Config.net.test.udp)
	r.callback = cb
	table.insert(in_channels, r)
	local s = si.new_publisher(Config.net.test.tcp)
	table.insert(out_channels, s)
	table.insert(ch_processing, procRaw)
end

local function shutdown()
  poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

poller = si.wait_on_channels(in_channels)
lut = poller.lut
poller:start()
