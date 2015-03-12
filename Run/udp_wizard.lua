#!/usr/bin/env luajit
dofile'../include.lua'
local Config = require'Config'
local si = require'simple_ipc'

if HOSTNAME=='alvin' or HOSTNAME=='teddy' then
	-- Don't forward
	os.exit()
end

local poller, lut
local in_channels = {}
local out_channels = {}
local function cb(skt)
	local ch_id = lut[skt]
	local in_ch = in_channels[ch_id]
	local out_ch = out_channels[ch_id]
	while in_ch:size() > 0 do out_ch:send(in_ch:receive()) end
end

for key,stream in pairs(Config.net.streams) do
	if type(stream.udp)=='number' then
		io.write('Forwarding ', key, ': ', stream.udp, ' -> ', stream.sub, '\n')
		local r = si.new_receiver(stream.udp)
		r.callback = cb
		table.insert(in_channels, r)
		local s = si.new_publisher(stream.sub)
		table.insert(out_channels, s)
	end
end
poller = si.wait_on_channels(in_channels)
lut = poller.lut
poller:start()
