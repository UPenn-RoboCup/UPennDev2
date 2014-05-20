dofile'include.lua'
local mp   = require'msgpack'
local util = require'util'
local vector = require'vector'
local simple_ipc = require'simple_ipc'
local wait_channels
local udp = require'udp'

local rpc_rep =
	simple_ipc.new_replier(Config.net.reliable_rpc,'*')
print('RPC | REP Receiving on',Config.net.reliable_rpc)
unix.usleep(1e5)
--
local rpc_sub =
	simple_ipc.new_subscriber(Config.net.reliable_rpc2, true, '*')
print('RPC | SUB Receiving on',Config.net.reliable_rpc2)
unix.usleep(1e5)
--
local rpc_udp =
	udp.new_receiver( Config.net.unreliable_rpc )
print('RPC | UDP Receiving on',Config.net.unreliable_rpc)

-- Require all necessary modules
require'vcm'
require'mcm'
require'hcm'
require'wcm'
Body = require'Body'

-- Require all necessary fsm channels
local fsm_channels = {}
for _,sm in ipairs(Config.fsm.enabled) do
  fsm_channels[sm] = simple_ipc.new_publisher(sm,true)
end

local function trim_string(str)
  if type(str)~='string' then return str end

  --SJ: with ubuntu, every string contains a byte 0 padding at the end  
   if str:byte(#str)==0 then
     str=string.sub(str,1,#str-1)      
   end
   return str
end

--NOTE: Can do memory AND fsm event.  In that order
local function process_rpc(rpc)
local status, reply
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
    status, reply = pcall(func,rpc.val)
    print('SHM |',rpc.shm,rpc.access,rpc.val,status,reply)
  end

  -- State machine events
  if rpc.fsm and type(rpc.evt)=='string' then
    local ch = fsm_channels[rpc.fsm]
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
      status, reply = pcall(func,body_args)
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

local function process_zmq(sh)
	local poll_obj = wait_channels.lut[sh]
  local request, has_more
  repeat
    request, has_more = poll_obj:receive(true)
		if request then
    	local rpc   = mp.unpack(request)
    	local reply = process_rpc(rpc)
    	-- if REQ/REP then reply
			if poll_obj.type=='rep' then
				local ret = rpc_rep:send( mp.pack(reply) )
			end
		end
  until not request
end

local function process_udp()
  while rpc_udp:size()>0 do
    local request = rpc_udp:receive()
    local rpc = mp.unpack(request)
    local reply = process_rpc(rpc)
    --print('Reply',reply)
  end
end

-- Send memory feedback
local feedback_udp_ch =
  udp.new_sender(Config.net.operator.wired, Config.net.feedback)

local function send_status_feedback()
  local data = {}
	data.pose = wcm.get_robot_pose()
	data.battery = Body.get_sensor_battery()
	data.rpy = Body.get_sensor_rpy()
  data.t   = unix.time()

  local ret, err = feedback_udp_ch:send( mp.pack(data) )
  --if err then print('Feedback UDP error',err) end
end

rpc_rep.callback = process_zmq
rpc_sub.callback = process_zmq
local rpc_udp_poll = {}
rpc_udp_poll.socket = rpc_udp:descriptor()
rpc_udp_poll.callback = process_udp
wait_channels = {rpc_rep,rpc_udp_poll,rpc_sub}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

-- For no feedback
--channel_poll:start()

local channel_timeout = 500 -- 2Hz joint feedback

--local channel_timeout = 50 -- 20Hz joint feedback
while true do
  local npoll = channel_poll:poll(channel_timeout)
  -- Send the feedback 
  send_status_feedback()
end
