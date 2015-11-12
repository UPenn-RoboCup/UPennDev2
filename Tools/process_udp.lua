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

local mattorch = require'mattorch'

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

	local t_in = {}
	local b_in = {}
	local t_out = {}
	local b_out = {}

	local burst = {}

	local tprev = -math.huge
	local bprev = -1
	for ii, pktsz in ipairs(use) do
		local tlog, bytesz = unpack(pktsz)
		local tdiff = tlog - tprev

		if (name:find'mesh0' or name:find'mesh1' or name:find'camera0' or name:find'camera1') and (bprev == bytesz and tdiff<=1) then
			if tlog >= 1433538150 then
				-- indoor only
				--if name:find'camera0' then print('Burst!', tdiff, bprev, bytesz) end
				burst[#burst] = (burst[#burst] or 0) + 1
			end
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
				--if (not name:find'camera') or tdiff >=1 then
					-- Inside!
					nb_in = nb_in + bytesz
					np_in = np_in + 1
					table.insert(t_in, tlog)
					table.insert(b_in, bytesz)
				--end
				if (name:find'mesh0' or name:find'mesh1' or name:find'camera0' or name:find'camera1') then
					--if name:find'camera0' then print(name, tdiff, 'Last burst', burst[#burst]) end
					burst[#burst+1] = 1
				end
			else
				-- Outside!
				nb_out = nb_out + bytesz
				np_out = np_out + 1
				table.insert(t_out, tlog)
				table.insert(b_out, bytesz)
			end
		end
		tprev = tlog
		bprev = bytesz
	end
	if not name:find'lidar' then
		print('\n= '..name..' =', #t_in, #t_out)
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
		mattorch.saveTable('/tmp/udp_'..name..'_trial1.mat', {
			t_in = t_in, b_in = b_in,
			t_out = t_out, b_out = b_out,
		})
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

	local t_in = {}
	local b_in = {}
	local t_out = {}
	local b_out = {}

	local burst = {0}

	local tprev = -math.huge
	local bprev = -1
	for ii, pktsz in ipairs(use) do
		local tlog, bytesz = unpack(pktsz)

		local tdiff = tlog - tprev
		if (name:find'mesh0' or name:find'mesh1' or name:find'camera0' or name:find'camera1') and (bprev == bytesz and tdiff<=1) then
			--print('Burst!', bytesz, tdiff)
			if tlog >= 1433623822.3194 then
				-- indoor only
				--if name:find'camera0' then print('Burst!', tdiff, bprev, bytesz) end
				burst[#burst] = (burst[#burst] or 0) + 1
			end
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
				--if (not name:find'camera') or tdiff >= 0.2 then
					-- Inside!
					nb_in = nb_in + bytesz
					np_in = np_in + 1
					table.insert(t_in, tlog)
					table.insert(b_in, bytesz)
				--end

				if (name:find'mesh0' or name:find'mesh1' or name:find'camera0' or name:find'camera1') then
					--if name:find'camera0' then
						--if burst[#burst]<3 then print(name, tdiff, 'Last burst', burst[#burst]) end
					--end
					burst[#burst+1] = 1
				end

			else
				-- Outside!
				nb_out = nb_out + bytesz
				np_out = np_out + 1
				table.insert(t_out, tlog)
				table.insert(b_out, bytesz)
			end
		end
		tprev = tlog
		bprev = bytesz
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
		print(nb_in / np_in / 1e3, 'kBytes per packet')
		--
		local nFull = 0
		local nTwo = 0
		for i, v in ipairs(burst) do
			if v>=3 then
				nFull = nFull + 1
			elseif v==2 then
				nTwo = nTwo + 1
			end
		end
		print('Full burst rate', 100*nFull / (#burst-1), nFull, (#burst-1) - nFull, 'nTwo', nTwo)
		mattorch.saveTable('/tmp/udp_'..name..'_trial2.mat', {
			t_in = t_in, b_in = b_in,
			t_out = t_out, b_out = b_out,
		})
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

local t_in = {}
local b_in = {}
local t_out = {}
local b_out = {}
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
		table.insert(t_out, t)
		table.insert(b_out, b)
	else
		-- indoor
		cmd_in = cmd_in + 1
		cmd_bytes_in = cmd_bytes_in + b
		table.insert(t_in, t)
		table.insert(b_in, b)
	end
end

mattorch.saveTable('/tmp/node_trial2.mat', {
	t_in = t_in, b_in = b_in,
	t_out = t_out, b_out = b_out,
})

print('out commands',cmd_out, cmd_bytes_out)
print('in  commands', cmd_in, cmd_bytes_in)
