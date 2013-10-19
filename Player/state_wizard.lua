dofile'include.lua'
local Body = require'Body'
local util = require'util'
local mp   = require'msgpack'
local signal = require'signal'
local simple_ipc   = require'simple_ipc'
local udp        = require'udp'

local state_pub_ch = simple_ipc.new_publisher(Config.net.state)

require'gcm'
require'wcm'
require'jcm'
require'mcm'

--SJ: This removes the output buffer 
io.stdout:setvbuf("no")

local use_joint_feedback = true
local needs_broadcast = true
local state_machines = {}

local status = {}

-- TODO: Make coroutines for each FSM
-- TODO: Or other way of handling state machine failure
-- Maybe a reset() function in each fsm?
for _,sm in ipairs(Config.fsm.enabled) do
  local my_fsm = require(sm)
  my_fsm.sm:set_state_debug_handle(function(cur_state_name,event)
    -- For other processes
    gcm['set_fsm_'..sm](cur_state_name)
    -- Local copy
    local s = {cur_state_name,event}
    status[my_fsm._NAME] = s
    -- Broadcast requirement
    needs_broadcast = true
    -- Debugging printing
    --print(table.concat(s,' from '))
  end)
  state_machines[sm] = my_fsm
  print( util.color('FSM | Loaded','yellow'),sm)
end

-- Update rate (if not webots)
local fps = 120
local us_sleep = 1e6 / fps

-- Start the state machines
local t0 = Body.get_time()
local t_debug = t0

--------------------
-- Clean Shutdown function
function shutdown()
  Body.exit()
  print'Shutting down the state machines...'
  for _,my_fsm in pairs(state_machines) do
    my_fsm.exit()
    -- Print helpful message
    print('Exit',my_fsm._NAME)
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local feedback_udp_ch
local function send_status_feedback()
  local data={};
  data.larmangle = Body.get_larm_command_position()
  data.rarmangle = Body.get_rarm_command_position()
  data.waistangle = Body.get_waist_command_position()
  data.neckangle = Body.get_head_command_position()
  data.llegangle = Body.get_lleg_command_position()
  data.rlegangle = Body.get_rleg_command_position()
  data.lgrip =  Body.get_lgrip_command_position()
  data.rgrip =  Body.get_rgrip_command_position()

  --Pose information
  data.pose =  wcm.get_robot_pose()    
  data.pose_odom =  wcm.get_robot_pose_odom()
  data.pose_slam =  wcm.get_slam_pose()
  data.rpy = Body.get_sensor_rpy()
  data.body_height = mcm.get_camera_bodyHeight()
  data.battery =  0

  local ret,err = feedback_udp_ch:send( mp.pack(data) )
  if err then print('feedback udp',err) end
end
if use_joint_feedback then
  feedback_udp_ch =
    udp.new_sender(Config.net.operator.wired,Config.net.feedback)
  print('Body | Status feedback Connected to Operator:',
    Config.net.operator.wired,Config.net.feedback)
end

-- Perform inialization
Body.entry()
for _,my_fsm in pairs(state_machines) do
  my_fsm.entry()
  local cur_state = my_fsm.sm:get_current_state()
  local cur_state_name = cur_state._NAME
  local s = {cur_state_name,nil}
  status[my_fsm._NAME] = s
  local ret = state_pub_ch:send( mp.pack(status) )
end


while true do
  local t = Body.get_time()
  
  -- Update each state machine
  for _,my_fsm in pairs(state_machines) do
    local event = my_fsm.update()
  end

  if needs_broadcast then
    needs_broadcast = false
    -- Broadcast over UDP/TCP/IPC
    local ret = state_pub_ch:send( mp.pack(status) )
  end
  
  -- Update the body (mostly needed for webots)
	Body.update()

  -- Send the joint state feedback
  if IS_WEBOTS or t-t_debug>1 then
    -- Webots debugs every step
    t_debug = t
    send_status_feedback()
  end
  
  -- Sleep a bit if not webots
  if not IS_WEBOTS then
    local t_loop = unix.time()-t
    local t_sleep = us_sleep-t_loop*1e6
    if t_sleep>0 then unix.usleep(t_sleep) end
  end
  
end
Body.exit()
for _,my_fsm in pairs(state_machines) do
  my_fsm.exit()
  local cur_state = my_fsm.sm:get_current_state()
  local cur_state_name = cur_state._NAME
  local s = {cur_state_name,nil}
  status[my_fsm._NAME] = s
  local ret = state_pub_ch:send( mp.pack(status) )
end
