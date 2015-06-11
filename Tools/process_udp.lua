#!/usr/bin/env luajit
dofile'../include.lua'
local mp = require'msgpack.MessagePack'

------------------------
local names = {
	'mesh0',
'ittybitty1',
'ittybitty0',
'lidar0',
'feedback',
'mesh1',
'lidar1',
'camera0',
'camera1'
}
local f_udp = io.open('Matlab/logs2/udp1433617367.9346.log')
local udp_use = f_udp:read('*all')
f_udp:close()
local udp_usage = mp.unpack(udp_use)
for i, name in ipairs(names) do
	local use = udp_usage[i]
	print(name, #use, 'received')
end

------------------------
local f_node_r = io.open('Matlab/logs1/node_recv.log')
local node_recv_str = f_node_r:read('*all')
f_node_r:close()
local node_recv = mp.unpack(node_recv_str)
print('Node received', #node_recv, 'items')
--for i, v in ipairs(node_recv) do print(unpack(v)) end

------------------------
local f_node_s = io.open('Matlab/logs1/node_sent.log')
local node_send_str = f_node_s:read('*all')
f_node_s:close()
local node_send = mp.unpack(node_send_str)
print('Node sent', #node_recv, 'items')
for i, v in ipairs(node_send) do print(unpack(v)) end
