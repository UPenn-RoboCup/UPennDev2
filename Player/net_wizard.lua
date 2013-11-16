---------------------------------
-- Network Monitor for Team THOR
-- (c) Stephen McGill
---------------------------------

dofile'include.lua'
require'unix'

local f, process_netstat
if OPERATING_SYSTEM=='darwin' then
  -- Mac
  f = io.popen'netstat -ib -I en3 1'
  process_netstat = function(s)
  	local stats = {}
  	for word in s:gmatch'%w+' do
  		if word=='input' or word=='packets' then return end
  		table.insert(stats,word)
  	end
  	local rx_bytes = tonumber(stats[3])
  	local tx_bytes = tonumber(stats[6])
    return rx_bytes,tx_bytes
  end
else
  -- Linux
  f = io.popen'stdbuf -i0 -o0 -e0 netstat -iec'
  process_netstat = function(s)
    local r_bytes = s:match'RX bytes:%d+'
    if r_bytes then
      local t_bytes = s:match'TX bytes:%d+'
      local rx_bytes = tonumber(r_bytes:sub(10))
      local tx_bytes = tonumber(t_bytes:sub(10))    
      if tx_bytes~=rx_bytes then return rx_bytes,tx_bytes end
    end
  end
end

-- Loop forever
local rx_bytes, tx_bytes
while true do
  local netstat = f:read()
  local t = unix.time()
  local rx,tx = process_netstat(netstat)
  if rx then print('rx',rx,'tx',tx) end
end