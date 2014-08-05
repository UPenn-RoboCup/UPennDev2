dofile'../fiddle.lua'
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
	--util.ptable(rpc)

  -- Shared memory modification
  if rpc.shm then
    local mem = _G[rpc.shm]
    if type(mem)~='table' then return'Invalid memory' end
    local func = mem[rpc.access]
    if type(func)~='function' then return'Invalid access function' end
    status, reply = pcall(func, rpc.val)
    print('SHM |',rpc.shm,rpc.access,rpc.val,status,reply)
  end

  -- State machine events
	local ch = rpc.fsm and type(rpc.evt)=='string' and fsm_chs[rpc.fsm]
	if ch then
		ch:send( rpc.special and {rpc.evt,rpc.special} or rpc.evt  )
		print('FSM |', rpc.fsm, rpc.evt, rpc.special)
	end
	--[[
  if rpc.fsm and type(rpc.evt)=='string' then
    local ch = fsm_chs[rpc.fsm]
    if ch then
      if rpc.special then
        ch:send{rpc.evt,rpc.special}
      else
        ch:send(rpc.evt)
      end
      print('FSM |',rpc.fsm,rpc.evt,rpc.special,status,reply)
    end
  end
	--]]

  -- Body functions
  local body_call = rpc.body
  local body_args = rpc.bargs
	local func = body_call and Body[body_call]
	if type(func)=='function' then
		status, reply = pcall(func, body_args)
		print('Body |',body_call,body_args,status,reply)
	end
	--[[
  if type(body_call)=='string' then
    local func = Body[body_call]
    if type(func)=='function' then
      status, reply = pcall(func, body_args)
      print('Body |',body_call,body_args,status,reply)
    end
  end
	--]]

  -- Raw commands
  local raw_call = rpc.raw
  if type(raw_call)=='string' then
    local res = loadstring('return '..raw_call)
    status, reply = pcall(res)
    print('RAW |',raw_call,status,reply)
  end

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