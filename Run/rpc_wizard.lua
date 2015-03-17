#!/usr/bin/env luajit
dofile'../fiddle.lua'
local ptable = require'util'.ptable
local ports = Config.net.rpc
local wait_channels = {}
-- Must reply to these TCP requests
print('REP TCP')
local tcp_rep = si.new_replier(ports.tcp_reply)
table.insert(wait_channels, tcp_rep)
print(ptable(tcp_rep))
-- Must reply to these UNIX domain socket requests
print('REP local')
local uds_rep = si.new_replier(ports.uds)
table.insert(wait_channels, uds_rep)
print(ptable(uds_rep))
-- Need not reply to these UDP requests
print('UDP')
local udp_sub = si.new_receiver(ports.udp)
table.insert(wait_channels, udp_sub)
print(ptable(udp_sub))
-- Setup the poller
local channel_poll
-- Gracefully exit on Ctrl-C
local function shutdown()
	io.write('\nShutting down the RPC handler')
	channel_poll:stop()
end
local signal = require'signal'
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

--NOTE: Can do memory AND fsm event.  In that order
local function process_rpc(rpc)
  local status, reply
  -- Debugging the request
	ptable(rpc)

  -- Shared memory modification
  if rpc.shm then
    local mem = _G[rpc.shm]
    if type(mem)~='table' then return'Invalid memory' end
		local seg = rpc.seg
    if type(seg)~='string' then return'Invalid memory segment' end
    local segment = mem[seg..'Keys']
    if type(rpc.key)~='string' then return'Invalid memory segment key' end
    local n_el = segment[rpc.key]
    if type(n_el)~='number' then return'Non-existent memory segment' end
    local val = rpc.val
    if val then
      local func = mem['set_'..seg..'_'..rpc.key]
      if type(func)~='function' then return'Invalid set function' end
      if type(val)=='table' then
				if #val~=n_el then return'Bad shm value' end
			elseif type(val)=='string' then
				if #val==0 then return'Bad shm val' end
			else
				return'Unsupported SHM value'
			end
      reply = pcall(func, val)
    else
      local func = mem['get_'..seg..'_'..rpc.key]
      if type(func)~='function' then return'Invalid get function' end
      status, reply = pcall(func)
    end
  end

  -- State machine events
  if rpc.fsm then
    if type(rpc.fsm)~='string' then return'Bad FSM' end
    if type(rpc.evt)~='string' then return'Bad FSM event' end
    local ch = fsm_chs[rpc.fsm..'FSM']
    if type(ch)~='table' or type(ch.send)~='function' then return'Bad FSM channel' end
    reply = ch:send(rpc.evt)
	end

  -- Body get/set parts functions
  if rpc.body then
    if type(rpc.body)~='string' then return'Bad body call' end
    local ids = Config.parts[rpc.body]
    if not ids then return'Not a body part' end
    if not rpc.comp then return'Missing Body component' end
    if type(rpc.comp)~='string' then return'Bad body component' end
    if rpc.val then
      if not dcm.actuatorKeys[rpc.comp] then return'Not a body component' end
      local func = Body['set_'..rpc.body:lower()..'_'..rpc.comp]
      if type(func)~='function' then return'Invalid set function' end
      if type(rpc.val)~='table' then return'Bad shm data' end
      if #rpc.val~=#ids then return'Bad shm length' end
      reply = pcall(func, rpc.val)
    else
      if not dcm.actuatorKeys[rpc.comp] and not dcm.sensorKeys[rpc.comp] then
        return'Not a body component'
      end
      local func = Body['get_'..rpc.body:lower()..'_'..rpc.comp]
      if type(func)~='function' then return'Invalid get function' end
      status, reply = pcall(func)
    end
	end

  -- TODO: Raw commands
  ----[[
  if type(rpc.raw)=='string' then
    local res = loadstring('return '..rpc.val)
    status, reply = pcall(res)
    --print('RAW |', rpc.raw, res, status, reply)
		print('RAW |', rpc.raw)
  end
  --]]

  return reply
end

local function process_zmq(skt)
	local requests, ch = skt:recv_all(), skt.obj
	local rpc, reply, ret
	for _, request in ipairs(requests) do
		rpc   = mp.unpack(request)
		reply = process_rpc(rpc)
		if ch.kind=='rep' then
			-- if REQ/REP then reply
			ret = ch:send(mp.pack(reply))
		end
	end
end

local function process_udp()
	local request, rpc, reply = rpc_udp:receive()
  while request do
    rpc = mp.unpack(request)
    reply = process_rpc(rpc)
    request = rpc_udp:receive()
  end
end

-- Assign the callback
tcp_rep.callback = process_zmq
uds_rep.callback = process_zmq
if udp_sub then
	udp_sub.callback = process_udp
end

-- Run the poller
channel_poll = si.wait_on_channels(wait_channels)
channel_poll:start()
