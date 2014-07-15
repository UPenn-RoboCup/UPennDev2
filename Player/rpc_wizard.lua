dofile'../fiddle.lua'
local mp   = require'msgpack'
local util = require'util'
local vector = require'vector'
local si = require'simple_ipc'
local udp = require'udp'
-- Must reply to these TCP requests
local rpc_rep =
	si.new_replier(Config.net.reliable_rpc)
print('RPC | REP Receiving on', Config.net.reliable_rpc)
--util.ptable(rpc_rep)
-- Need not reply to these TCP requests
local rpc_sub =
	si.new_subscriber(Config.net.reliable_rpc2)
print('RPC | SUB Receiving on', Config.net.reliable_rpc2)
--util.ptable(rpc_sub)
-- Need not reply to these UDP requests
local rpc_udp =
	udp.new_receiver(Config.net.unreliable_rpc)
print('RPC | UDP Receiving on', Config.net.unreliable_rpc)

-- SJ: with ubuntu, every string contains a byte 0 padding at the end
local function trim_string(str)
	if type(str)=='string' and str:byte(#str)==0 then
		str = string.sub(str, 1, #str-1)
	end
	return str
end

--NOTE: Can do memory AND fsm event.  In that order
local function process_rpc(rpc)
  local status, reply
  -- Debugging the request
	--util.ptable(rpc)

  -- TODO: Remove the stupid trim_string necessity
  rpc.fsm = trim_string(rpc.fsm)
  rpc.evt = trim_string(rpc.evt)
  rpc.special = trim_string(rpc.special)
  --
  rpc.shm = trim_string(rpc.shm)
  rpc.access = trim_string(rpc.access)
  rpc.val = trim_string(rpc.val)
  --
  rpc.body = trim_string(rpc.body)
  rpc.bargs = trim_string(rpc.bargs)
  -- Experimental raw support
  rpc.raw = trim_string(rpc.raw)

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

  -- Body functions
  local body_call = rpc.body
  local body_args = rpc.bargs
  if type(body_call)=='string' then
    local func = Body[body_call]
    if type(func)=='function' then
      status, reply = pcall(func, body_args)
      print('Body |',body_call,body_args,status,reply)
    end
  end

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
	print("HERE")
	local requests, obj = skt:recv_all(), skt.obj
	local skt_kind = obj.kind
	for _, request in ipairs(requests) do
		local rpc   = mp.unpack(request)
		local reply = process_rpc(rpc)
		if skt_kind=='rep' then
			-- if REQ/REP then reply
			local ret = rpc_rep:send(mp.pack(reply))
		end
	end
end

local function process_udp()
  while rpc_udp:size()>0 do
    local request = rpc_udp:receive()
    local rpc = mp.unpack(request)
    local reply = process_rpc(rpc)
    --print('Reply',reply)
  end
end

rpc_rep.callback = process_zmq
rpc_sub.callback = process_zmq
local rpc_udp_poll = {
	socket = rpc_udp:descriptor(),
	callback = process_udp,
}
local wait_channels = {rpc_rep, rpc_udp_poll, rpc_sub}
local channel_poll = si.wait_on_channels(wait_channels)

-- For no feedback
--channel_poll:start()

-- Send periodic memory feedback
local feedback_udp_ch = udp.new_sender(Config.net.operator.wired, Config.net.feedback)
local function send_status_feedback()
  local data = {
		pose = wcm.get_robot_pose(),
		battery = Body.get_sensor_battery(),
		rpy = Body.get_sensor_rpy(),
  	t   = unix.time(),
	}
  local ret, err = feedback_udp_ch:send( mp.pack(data) )
  --if err then print('Feedback UDP error',err) end
end

local channel_timeout = 500 -- 2Hz joint feedback
while true do
  local npoll = channel_poll:poll(channel_timeout)
  -- Send the feedback
	if ENABLE_FEEDBACK then
	  send_status_feedback()
	end
end
