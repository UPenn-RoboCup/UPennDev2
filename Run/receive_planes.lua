#!/usr/bin/env luajit
pcall(dofile, 'include.lua')
pcall(dofile, '../include.lua')
local si = require'simple_ipc'
local ptable = require'util'.ptable
local munpack = require'msgpack.MessagePack'.unpack

print(Config.net.field_computer)
local planes_ch = si.new_subscriber(49000, Config.net.field_computer)
ptable(planes_ch)

function planes_ch.callback()
	local mplanes = unpack(planes_ch:receive())
	local planes = munpack(mplanes)
	ptable(planes)
end

-- Cleanly exit on Ctrl-C
local running = true
local function shutdown()
	print('Shutdown!')
	poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local poller = si.wait_on_channels{planes_ch}
poller:start()
