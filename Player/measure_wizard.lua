local do_log = false
local ntimes = 240
local rate = 0.5

dofile'include.lua'
require'unix'
mp = require'msgpack'
util = require'util'
fname = 'load.log'
torch = require'torch'
torch.Tensor = torch.DoubleTensor
--[[
local header_ns
local function get_net()
	local ns = io.popen'netstat -i'
	ns:read()
	local nsh = ns:read()
	if not header_ns then
		header_ns = {}
		local h = nsh:gmatch('%S+')
		local idx = 1
		for v in h do
			header_ns[v] = idx
			idx = idx+1
		end
		--	util.ptable(header_ns)
	end

	local rx0,tx0
	while true do
		local d = ns:read()
		if not d then break end
		if d:find('eth2%s+') then
			local dd = d:gmatch('%S+')
			local tmp = {}
			for v in dd do table.insert(tmp,v) end
			rx0 = tmp[header_ns['RX-OK'] ]
			tx0 = tmp[header_ns['TX-OK'] ]
			break
		end
	end
	return rx0,tx0
end
--]]

local function get_net()
	local rx,tx
	local ns = io.popen'ifconfig eth2'
	local data = ns:read('*all')
	rx = tonumber( data:match('RX bytes:(%d+)') )
	tx = tonumber( data:match('TX bytes:(%d+)') )
	return rx,tx
end

local function log()
	local lut = {}

	local top_cmd = 'top -b -d '..rate..' -n '..ntimes
	local pids = io.popen'ps h -C lua -o cmd,pid'
	while true do
		local pid = pids:read()
		if not pid then break end
		local process = pid:gmatch('lua (%S*)_wizard%.lua%s*(%d+)')
		local a,b = process()
		lut[b] = a
		--lut[a] = b
		top_cmd = top_cmd..' -p'..b
	end
	--print(top_cmd)

	local fl = io.open('load.log','w')
	fl:write(mp.pack(top_cmd))
	fl:write(mp.pack{ntimes=ntimes,rate=rate})
	fl:write(mp.pack(lut))
	local top = io.popen(top_cmd)
	local header
	local data
	local count = 0
	while true do
		local x = top:read()
		if not x then break end
		--print(x)
		-- Check if new cycle
		local new_cycle = false
		if x:find'top'==1 then
			count = count + 1
			print('Count',count,'of',ntimes)
			-- Write the last log
			if data then
				--util.ptable(data)
				--util.ptable(data.lidar)
				fl:write(mp.pack(data))
			end
			local rx, tx = get_net()
			data = {t=unix.time(),n=count,rx=rx,tx=tx}
			--print('top',x)
		elseif x:find'%d+%s+thor' then
			--local d = x:gmatch('%d+[:%d*]*[%.%d*]*')
			local d = x:gmatch('%S+')
			--print('data',x)
			local dd = {}
			for v in d do table.insert(dd,v) end
			--util.ptable(dd)
			local pid = dd[header['PID']]
			data[pid] = {
				cpu = tonumber(dd[header['%CPU']]),
				mem = tonumber(dd[header['%MEM']]),
			}
		elseif x:find'PID' then
			--print('header',x)
			if not header then
				header = {}
				local h = x:gmatch('%S+')
				local idx = 1
				for v in h do
					header[v] = idx
					idx = idx+1
				end
			end -- no header
		end -- check type of reow
	end -- inf while
	fl:write(mp.pack(data))
	fl:close()

end

local function unlog()
	local fl = io.open('bad_net_load.log','r')
	--local fl = io.open('good_net_load.log','r')
	local unpacker = msgpack.unpacker(fl:read('*a'))
	fl:close()
	--
	local top = unpacker:unpack()
  print('Top:',top)
	--
	local settings = unpacker:unpack()
	local n = settings.ntimes
	local r = settings.rate
  print('Tracked',n,r)
	--
	local lut = unpacker:unpack()
	local nproc = util.tablesize(lut)
  print('Processes',nproc)
  util.ptable(lut)
	local ids, id = {}, 0
	for k,v in pairs(lut) do
		id = id + 1
		--ids[k] = id
		ids[v] = id
	end
	--
	local cpu = torch.Tensor(nproc,n)
	local mem = torch.Tensor(nproc,n)
	local ts = torch.Tensor(n)
	local tx = torch.Tensor(n)
	local rx = torch.Tensor(n)
	--
	local cnt = 0
	local tbl = true
	while tbl do
		tbl = unpacker:unpack()
		if not tbl then break end
		cnt = cnt+1
		ts[cnt] = tbl.t
		print(tbl.tx,type(tbl.tx))
		tx[cnt] = tbl.tx
		rx[cnt] = tbl.rx
		for k,v in pairs(tbl) do
			if type(v)=='table' then
				local id = ids[k]
				cpu[id][cnt] = v.cpu
				mem[id][cnt] = v.mem
			end
		end
	end

	-- Print out simple statistic
	local ts_u = ts:sub(2,ts:size(1))
	local ts_l = ts:sub(1,ts:size(1)-1)
	local ts_diff = ts_u - ts_l
	--
	local tx_u = tx:sub(2,tx:size(1))
	local tx_l = tx:sub(1,tx:size(1)-1)
	local tx_diff = tx_u - tx_l
	--
	local rx_u = rx:sub(2,rx:size(1))
	local rx_l = rx:sub(1,rx:size(1)-1)
	local rx_diff = rx_u - rx_l
	--
	local tx_rate = torch.cdiv(tx_diff,ts_diff)
	local rx_rate = torch.cdiv(tx_diff,ts_diff)
	-- Save rates to file
	--
	local total_time = ts[-1]-ts[1]
	local tx_rate_total = (tx[-1]-tx[1]) / total_time
	local rx_rate_total = (rx[-1]-rx[1]) / total_time
	print('Rates',tx_rate_total,rx_rate_total)
	
  
	for k,v in pairs(ids) do
		print('\n'..k)
		local data = cpu:select(1,v)
		local std  = torch.std(data)
		local mean = torch.mean(data)
		print('CPU:',mean,std)
		local data = mem:select(1,v)
		local std  = torch.std(data)
		local mean = torch.mean(data)
		print('MEM:',mean,std)
	end

end
if do_log then log() else unlog() end
