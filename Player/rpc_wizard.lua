#!/usr/bin/env luajit
dofile'../fiddle.lua'
local ptable = require'util'.ptable
local ports = Config.net.rpc
-- Must reply to these TCP requests
local tcp_rep = si.new_replier(ports.tcp_reply)
print('RPC | REP Receiving on', ports.tcp_reply)
-- Need not reply to these TCP requests
local tcp_sub = si.new_subscriber(ports.tcp_subscribe)
print('RPC | SUB Receiving on', ports.tcp_subscribe)
-- Need not reply to these UDP requests
local udp_sub = si.new_receiver(ports.udp)
print('RPC | UDP Receiving on', ports.udp)
-- Must reply to these UNIX domain socket requests
local uds_rep = si.new_replier(ports.uds)
print('RPC | REP Receiving on', ports.uds)
-- Setup the poller
local wait_channels = {tcp_rep, tcp_sub, udp_sub, uds_rep}
local channel_poll
-- Gracefully exit on Ctrl-C
local function shutdown()
	print('\nShutting down the RPC handler')
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
    local seg = rpc.seg
    if type(mem)~='table' then return'Invalid memory' end
    if type(seg)~='string' then return'Invalid memory segment' end
    local segment = mem[seg..'Keys']
    if type(rpc.key)~='string' then return'Invalid memory segment key' end
    local n_el = segment[rpc.key]
    if type(n_el)~='number' then return'Non-existent memory segment' end
    local val = rpc.val
    if val then
      local func = mem['set_'..seg..'_'..rpc.key]
      if type(func)~='function' then return'Invalid set function' end
      if type(val)~='table' then return'Bad shm data' end
      if  #val~=n_el then return'Bad shm length' end
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
  --[[
  local raw_call = rpc.raw
  if type(raw_call)=='string' then
    local res = loadstring('return '..raw_call)
    status, reply = pcall(res)
    print('RAW |', raw_call, status, reply)
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
tcp_sub.callback = process_zmq
uds_rep.callback = process_zmq
udp_sub.callback = process_udp

-- Run the poller
channel_poll = si.wait_on_channels(wait_channels)
channel_poll:start()
