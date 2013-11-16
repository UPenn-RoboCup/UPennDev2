---------------------------------
-- State Machine Manager for Team THOR
-- (c) Stephen McGill
---------------------------------
dofile'include.lua'
local Body = require'Body'
local util = require'util'
local mp   = require'msgpack'
local signal = require'signal'
local simple_ipc = require'simple_ipc'
local udp        = require'udp'
local vector     = require'vector'

-- Not using this right now...
local broadcast_en = false
local state_pub_ch
if broadcast_en then
  state_pub_ch = simple_ipc.new_publisher(Config.net.state)
end
local pulse_ch = simple_ipc.new_publisher'pulse'

require'gcm'
require'wcm'
require'jcm'
require'mcm'

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

-- Perform inialization
Body.entry()
for _,my_fsm in pairs(state_machines) do
  my_fsm.entry()
  local cur_state = my_fsm.sm:get_current_state()
  local cur_state_name = cur_state._NAME
  local s = {cur_state_name,nil}
  status[my_fsm._NAME] = s
  if broadcast_en then
    local ret = state_pub_ch:send( mp.pack(status) )
  end
end

-- Joint reading... seems that we are always reading????
local all_joint_read = vector.ones(#jcm.get_read_position())

-- Pulse
local pulse_tbl = {t=0}

while true do
  local t = Body.get_time()
  
  -- Update each state machine
  for _,my_fsm in pairs(state_machines) do local event = my_fsm.update() end

  -- Broadcast state changes... why...? Need the stage as well?
  if broadcast_en and needs_broadcast then
    needs_broadcast = false
    -- Broadcast over UDP/TCP/IPC
    local ret = state_pub_ch:send( mp.pack(status) )
  end
  
  -- Always read from all the motors
  jcm.set_read_position(all_joint_read)
  
  -- Update the body (mostly needed for webots)
	Body.update()
  
  -- Send the tick tock at the end of a cycle
  pulse_tbl.t = t
  pulse_ch:send(mp.pack(pulse_tbl))
  
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
