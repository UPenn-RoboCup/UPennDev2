dofile'include.lua'
local mp   = require'msgpack'
local util = require'util'
local vector = require'vector'
local simple_ipc = require'simple_ipc'

-- TODO: Use the Config file for the ports
local udp = require'udp'
local rpc_zmq = simple_ipc.new_replier(Config.net.reliable_rpc,'*')
unix.usleep(1e5)
local rpc_udp = udp.new_receiver( Config.net.unreliable_rpc )
print('RPC | Receiving on',Config.net.reliable_rpc)

-- TODO: Require all necessary modules
require'vcm'
require'jcm'
require'mcm'
require'hcm'

-- New Body API
require'Body'

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
  -- for debugging
  util.ptable(rpc)
  rpc.fsm = trim_string(rpc.fsm)
  rpc.evt = trim_string(rpc.evt)
  rpc.special = trim_string(rpc.special)
  --
  rpc.shm = trim_string(rpc.shm)
  rpc.segment = trim_string(rpc.segment)
  rpc.key = trim_string(rpc.key)
  rpc.segkeyfun = trim_string(rpc.segkey) -- segment AND key AND function
  --
  rpc.body = trim_string(rpc.body)
  rpc.bargs = trim_string(rpc.bargs)
  -- Experimental raw support
  rpc.raw = trim_string(rpc.raw)

  local status, reply
  -- Shared memory modification
  local shm = rpc.shm
  if shm then
    local mem = _G[shm]
    if type(mem)~='table' then return'Invalid memory' end

    if rpc.segkeyfun then
      local func = mem[rpc.segkeyfun]
      -- Use a protected call
      print(rpc.segkeyfun)
      status, reply = pcall(func,rpc.val)
    elseif rpc.val then
      -- Set memory
      local method = string.format('set_%s_%s',rpc.segment,rpc.key)
      local func = mem[method]
      -- Use a protected call
      print(method)
      status, reply = pcall(func,rpc.val)
    elseif rpc.delta then
      -- Increment/Decrement memory
      local method = string.format('_%s_%s',rpc.segment,rpc.key)
      local func = mem['get'..method]
      status, cur = pcall(func)
      func = mem['set'..method]
      local up = vector.new(cur)+vector.new(rpc.delta)
      status, reply = pcall(func,up)
    else
      -- Get memory
      local method = string.format('get_%s_%s',rpc.segment,rpc.key)
      local func = mem[method]
      -- Use a protected call
      status, reply = pcall(func)
    end
  end -- if shm

  -- State machine events
  local fsm = rpc.fsm
  if fsm and type(rpc.evt)=='string' then
    local ch = fsm_channels[fsm]
    if ch then
      if rpc.special then
        ch:send{rpc.evt,rpc.special}
      else
        ch:send(rpc.evt)
      end
    end
  end
  
  -- Body
  local body_call = rpc.body
  local body_args = rpc.bargs
  if type(body_call)=='string' then
    status, reply = pcall(Body[body_call],body_args)
    print('body',body_call,body_args,status,reply)
  end
  
  -- Raw commands
  local raw_call = rpc.raw
  if type(raw_call)=='string' then
    local res = loadstring('return '..raw_call)
    reply = res()
    print('RAW | ',raw_call,reply)
  end

  return reply
end

local function process_zmq()
  local request, has_more
  repeat
    request, has_more = rpc_zmq:receive()
    local rpc         = mp.unpack(request)
    local reply       = process_rpc(rpc)
    -- NOTE: The zmq channel is REP/REQ
    -- Reply with the result of the request
    local ret         = rpc_zmq:send( mp.pack(reply) )
  until not has_more
end

local function process_udp()
  while rpc_udp:size()>0 do
    local request = rpc_udp:receive()
    local rpc = mp.unpack(request)
    local reply = process_rpc(rpc)
    --print('Reply',reply)
  end
end

-- Send body feedback
local Body = require'Body'
local feedback_udp_ch =
  udp.new_sender(Config.net.operator.wired, Config.net.feedback)
local function send_status_feedback()
  local data={};
  data.larmangle  = Body.get_larm_command_position()
  data.rarmangle  = Body.get_rarm_command_position()
  data.waistangle = Body.get_waist_command_position()
  data.neckangle  = Body.get_head_command_position()
  data.llegangle  = Body.get_lleg_command_position()
  data.rlegangle  = Body.get_rleg_command_position()
  data.lgrip = Body.get_lgrip_command_position()
  data.rgrip = Body.get_rgrip_command_position()
  -- Gripper
  data.l_load = Body.get_lgrip_load()
  data.r_load = Body.get_rgrip_load()
  data.l_temp = Body.get_lgrip_temperature()
  data.r_temp = Body.get_rgrip_temperature()
  data.l_gpos = Body.get_lgrip_position()
  data.r_gpos = Body.get_rgrip_position()
  

  --Pose information
--  data.pose =  wcm.get_robot_pose()    

--SJ: now we apply torso compensation to pose 
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  data.pose = util.pose_global(
    vector.new({uTorsoComp[1],uTorsoComp[2],0}), wcm.get_robot_pose()
    )

  data.pose_odom =  wcm.get_robot_pose_odom()
  data.pose_slam =  wcm.get_slam_pose()
  data.rpy = Body.get_sensor_rpy()
  data.body_height = mcm.get_stance_bodyHeight()
  data.battery =  0
  data.t = Body.get_time()

  local ret,err = feedback_udp_ch:send( mp.pack(data) )
--  if err then print('feedback udp',err) end
end

rpc_zmq.callback = process_zmq
local rpc_udp_poll = {}
rpc_udp_poll.socket_handle = rpc_udp:descriptor()
rpc_udp_poll.callback = process_udp
local wait_channels = {rpc_zmq,rpc_udp_poll}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
--channel_poll:start()
local channel_timeout = 500 -- 2Hz joint feedback

--local channel_timeout = 50 -- 20Hz joint feedback
while true do
  local npoll = channel_poll:poll(channel_timeout)
  -- Send the feedback 
  send_status_feedback()
end
