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

print('\nTrial 1')
local f_udp = io.open('Matlab/logs1/udp1433560169.5507.log')
local udp_use = f_udp:read('*all')
f_udp:close()
local udp_usage = mp.unpack(udp_use)
for i, name in ipairs(names) do
	local use = udp_usage[i]
	local nbytes = 0
	local npkts = 0

	local np_in, nb_in = 0,0
	local np_out, nb_out = 0,0

	local tprev = -math.huge
	for ii, pktsz in ipairs(use) do
		local tlog, bytesz = unpack(pktsz)

		local tdiff = tlog - tprev
		tprev = tlog
		if tdiff < 1e-3 then
			--print('Burst!')
		elseif tlog<1433536200 then
		elseif tlog>1433539557 then
		elseif tlog>=1433537480 then -- this is after that dumb bug :P true measurement
			-- Actual start (after the car?)
		--if tlog>=1433623621.0684 then
			nbytes = nbytes + bytesz
			npkts = npkts + 1
		--else print('skip!', tlog)

			--if name:find'camera0' then print(name, tlog, tlog - 1433623675.096) end
			--if name:find'mesh0' then print(name, tlog, tlog - 1433623675.096) end
			--if name:find'itty' then print(name, tlog, tlog - 1433623675.096) end



		if tlog >= 1433538150 then
			-- Inside!
			nb_in = nb_in + bytesz
			np_in = np_in + 1
		else
			-- Outside!
			nb_out = nb_out + bytesz
			np_out = np_out + 1
		end

		end
	end
	if not name:find'lidar' then
		print('\n= '..name..' =')
		print(nbytes, 'bytes')
		print(nbytes/1000, 'kbytes')
		print(nbytes/1000/1000, 'Mbytes')
		print(npkts, 'packets')
		--
		print(nb_out, 'bytes outside')
		print(nb_out/1000, 'kbytes outside')
		print(nb_out/1000/1000, 'Mbytes outside')
		print(np_out, 'packets outside')
		--
		print(nb_in, 'bytes inside')
		print(nb_in/1000, 'kbytes inside')
		print(nb_in/1000/1000, 'Mbytes inside')
		print(np_in, 'packets inside')
	end
end
print()
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
--print('Node sent', #node_recv, 'items')
local t0 = node_send[1][1]
local cmd_in, cmd_out = 0,0
local cmd_bytes_out, cmd_bytes_in = 0, 0
for i, v in ipairs(node_send) do
	local t, b = unpack(v)
	-- At the door: 1433623646.0488
	-- Door swings open: 1433623732.4492
	-- Begin walking through the door: 1433623770.537
	-- Inside conditions: 1433623822.3194
	-- Move arm to the valve: 1433624078.416
	-- Aligned to the valve: 1433624212.348
	-- Done with SJ's low level control to turn it: 1433624312.668
	-- Back from the valve: 1433624372.5473
	--print( t/1e3, (t-t0)/1e3, b, t/1e3 - 1433538627.495)
	if t/1e3 < 1433538150 then
		--outdoor
		cmd_out = cmd_out + 1
		cmd_bytes_out = cmd_bytes_out + b
	elseif t/1e3 < 1433539557 then
		-- indoor
		cmd_in = cmd_in + 1
		cmd_bytes_in = cmd_bytes_in + b
	end

end

print('Node out commands',cmd_out, cmd_bytes_out)
print('Node in  commands', cmd_in, cmd_bytes_in)

print('\n\n\n\n\n')

--[[
-- maybe trial1?
print('\nTrial 1 Maybe')
local f_udp = io.open('Matlab/logs2/udp1433560169.5507.log')
local udp_use = f_udp:read('*all')
f_udp:close()
local udp_usage = mp.unpack(udp_use)
for i, name in ipairs(names) do
	local use = udp_usage[i]
	print(name, #use, 'received')
end

-- trial2 oops start
print('\nTrial 2 pre-start')
local f_udp = io.open('Matlab/logs2/udp1433617367.9346.log')
local udp_use = f_udp:read('*all')
f_udp:close()
local udp_usage = mp.unpack(udp_use)
for i, name in ipairs(names) do
	local use = udp_usage[i]
	print(name, #use, 'received')
end
--]]

-- Correct trial2 results
print('\nTrial2 Received data')
local f_udp = io.open('Matlab/logs2/udp1433626360.5306.log')
local udp_use = f_udp:read('*all')
f_udp:close()
local udp_usage = mp.unpack(udp_use)
for i, name in ipairs(names) do
	local use = udp_usage[i]
	local nbytes = 0
	local npkts = 0

	local np_in, nb_in = 0,0
	local np_out, nb_out = 0,0

	local tprev = -math.huge
	for ii, pktsz in ipairs(use) do
		local tlog, bytesz = unpack(pktsz)

		local tdiff = tlog - tprev
		tprev = tlog
		if tdiff < 1e-3 then
			--print('Burst!')
		elseif tlog>=1433623675.096 then -- this is after that dumb bug :P true measurement
			-- Actual start (after the car?)
		--if tlog>=1433623621.0684 then
			nbytes = nbytes + bytesz
			npkts = npkts + 1
		--else print('skip!', tlog)

			--if name:find'camera0' then print(name, tlog, tlog - 1433623675.096) end
			--if name:find'mesh0' then print(name, tlog, tlog - 1433623675.096) end
			--if name:find'itty' then print(name, tlog, tlog - 1433623675.096) end



		if tlog >= 1433623822.3194 then
			-- Inside!
			nb_in = nb_in + bytesz
			np_in = np_in + 1
		else
			-- Outside!
			nb_out = nb_out + bytesz
			np_out = np_out + 1
		end

		end
	end
	if not name:find'lidar' then
		print('\n= '..name..' =')
		print(nbytes, 'bytes')
		print(nbytes/1000, 'kbytes')
		print(nbytes/1000/1000, 'Mbytes')
		print(npkts, 'packets')
		--
		print(nb_out, 'bytes outside')
		print(nb_out/1000, 'kbytes outside')
		print(nb_out/1000/1000, 'Mbytes outside')
		print(np_out, 'packets outside')
		--
		print(nb_in, 'bytes inside')
		print(nb_in/1000, 'kbytes inside')
		print(nb_in/1000/1000, 'Mbytes inside')
		print(np_in, 'packets inside')
	end
end

print()
------------------------
local f_node_r = io.open('Matlab/logs2/node_recv.log')
local node_recv_str = f_node_r:read('*all')
f_node_r:close()
local node_recv = mp.unpack(node_recv_str)
print('Node received', #node_recv, 'items')
--for i, v in ipairs(node_recv) do print(unpack(v)) end

------------------------
local f_node_s = io.open('Matlab/logs2/node_sent.log')
local node_send_str = f_node_s:read('*all')
f_node_s:close()
local node_send = mp.unpack(node_send_str)
print('Node sent', #node_recv, 'items')
local t0 = node_send[1][1]
local cmd_in, cmd_out = 0,0
local cmd_bytes_out, cmd_bytes_in = 0, 0
for i, v in ipairs(node_send) do
	local t, b = unpack(v)
	-- At the door: 1433623646.0488
	-- Door swings open: 1433623732.4492
	-- Begin walking through the door: 1433623770.537
	-- Inside conditions: 1433623822.3194
	-- Move arm to the valve: 1433624078.416
	-- Aligned to the valve: 1433624212.348
	-- Done with SJ's low level control to turn it: 1433624312.668
	-- Back from the valve: 1433624372.5473
	--print( t/1e3, (t-t0)/1e3, b, t/1e3 - 1433623768.4502)
	if t/1e3 < 1433623822.3194 then
		--outdoor
		cmd_out = cmd_out + 1
		cmd_bytes_out = cmd_bytes_out + b
	else
		-- indoor
		cmd_in = cmd_in + 1
		cmd_bytes_in = cmd_bytes_in + b
	end

end

print('out commands',cmd_out, cmd_bytes_out)
print('in  commands', cmd_in, cmd_bytes_in)
