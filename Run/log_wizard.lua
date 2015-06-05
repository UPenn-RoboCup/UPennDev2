#!/usr/bin/env luajit
dofile'../include.lua'
local Config = require'Config'
local si = require'simple_ipc'
local util = require'util'
local unzlib = require'zlib.ffi'.uncompress
local libLog = require'libLog'

local mpack = require'msgpack.MessagePack'.pack
local munpack = require'msgpack.MessagePack'.unpack

local poller, lut
local channels = {}
local ch_names = {}
local loggers = {}
local log_times = {}

local signal = require'signal'.signal
local function shutdown()
  poller:stop()
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local function cb(skt)
	local ch_id = lut[skt]
	local ch = channels[ch_id]
	local mdata, payload = unpack(skt:recv_all())
	local t = unix.time()

	-- Only log at 2Hz max
	if t - log_times[ch_id] < 0.5 then return end

	local meta = munpack(mdata)
	print(data, type(data)=='table' and #table)
	if loggers[ch_id].n % 10 then print('Logging '..ch_names[ch_id], loggers[ch_id].n) end
	if loggers[ch_id].n % 100 == 0 then loggers[ch_id] = libLog.new(ch_names[ch_id], true) end
	metadata.tlog = t
	metadata.rsz = #payload
	loggers[ch_id]:record(meta, payload, #payload)
	log_times[ch_id] = t
end

-- This is on the operator side
for key,stream in pairs(Config.net.streams) do
	if type(stream.udp)=='number' then
		print('Logging', key)
		local r = si.new_subscriber(stream.sub)
		r.callback = cb
		table.insert(in_channels, r)
		table.insert(ch_names, key)
		table.insert(loggers, libLog.new(key, true))
	end
end

-- This is on the field computer side
if unix.gethostname()=='surge' then
	for key,stream in pairs(Config.net.streams) do
		if type(stream.tcp)=='number' then
			print('Logging', key)
			local r = si.new_subscriber(stream.tcp)
			r.callback = cb
			table.insert(in_channels, r)
			table.insert(ch_names, key)
			table.insert(loggers, libLog.new(key, true))
		end
	end
end

poller = si.wait_on_channels(channels)
lut = poller.lut
poller:start()

for i, logger in ipairs(loggers) do logger:stop() end
