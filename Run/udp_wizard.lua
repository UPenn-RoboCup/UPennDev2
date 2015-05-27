#!/usr/bin/env luajit
dofile'../include.lua'
local Config = require'Config'
local si = require'simple_ipc'
local util = require'util'
local unzlib = require'zlib'.inflate()
local unzlib = require'zlib.ffi'.uncompress

local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack
local function procMP(data)
	if not type(data)=='table' then return end
	local ok, tbl, offset = pcall(munpack, data)
	if not ok then return end
	if not type(tbl)=='table' then return end
	if not type(offset)=='number' then return end
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

local function procZlib(c_data)
	print(#c_data)
	local data = unzlib(c_data)
	if not data then return end
	return procMP(data)
end

local poller, lut
local in_channels = {}
local out_channels = {}
local ch_processing = {}
local ch_usage = {}
local ch_names = {}
local function cb(skt)
	local ch_id = lut[skt]
	local in_ch = in_channels[ch_id]
	local out_ch = out_channels[ch_id]
	local usage = ch_usage[ch_id]
	
	local fn = ch_processing[ch_id]
	local sz = in_ch:size()
	local data
	while sz > 0 do
		data = in_ch:receive()
		table.insert(usage, {unix.time(), #data})
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
		if key=='feedback' then
			table.insert(ch_processing, procZlib)
		else
			table.insert(ch_processing, procMP)
		end
		table.insert(ch_usage, {})
		table.insert(ch_names, key)
	end
end

local ping_ch, go_ch
if IS_COMPETING then
	local NET_OPEN = false
	ping_ch = si.new_publisher(Config.net.ping.tcp)
	go_ch = si.new_receiver(Config.net.ping.udp)
	table.insert(out_channels, ping_ch)
	table.insert(in_channels, go_ch)
	go_ch.callback = function(skt)
		local ch_id = lut[skt]
		local in_ch = in_channels[ch_id]
		local out_ch = out_channels[ch_id]
		local sz = in_ch:size()
		while sz > 0 do
			local data = in_ch:receive()
			out_ch:send('ok')
			sz = in_ch:size()
		end
	end
end

function net()
	for ch_id, usage in ipairs(ch_usage) do
		local sum = 0
		for i, use in ipairs(usage) do
			local t_use, bytes_used = unpack(use)
			sum = sum + bytes_used
		end
		print(string.format('Received %d bytes from %s', sum, ch_names[ch_id]))
	end
end

local function shutdown()
	local f = io.open('udp'..unix.time()..'.log', 'w')
	f:write(mpack(ch_usage))
	f:close()
	net()
  poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

poller = si.wait_on_channels(in_channels)
lut = poller.lut
poller:start()
