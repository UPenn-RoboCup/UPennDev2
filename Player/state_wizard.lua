dofile'include.lua'
local Body = require'Body'
local util = require'util'
local mp   = require'msgpack'
local signal = require'signal'
local simple_ipc   = require'simple_ipc'
local state_pub_ch = simple_ipc.new_publisher(Config.net.state)

require'gcm'
require'jcm'

--SJ: This removes the output buffer 
io.stdout:setvbuf("no")


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
  
  -- Sleep a bit if not webots
  if not IS_WEBOTS then
    local t_loop = unix.time()-t
    --io.flush(stdout)
    --unix.usleep(us_sleep-t_loop*1e6)
    unix.usleep(us_sleep)
  end

  -- Debugging
  if t-t_debug>1 then
    t_debug = t
    local debug_tbl = {}
    table.insert(debug_tbl,string.format(
      'RPY:  %s', tostring(Body.get_sensor_rpy())
      ))
    table.insert(debug_tbl,string.format(
      'Gyro: %s', tostring(Body.get_sensor_gyro())
      ))
    table.insert(debug_tbl,string.format(
      'LLeg: %s', tostring(Body.get_lleg_position())
      ))
    table.insert(debug_tbl,string.format(
      'RLeg: %s', tostring(Body.get_rleg_position())
      ))
    print(table.concat(debug_tbl,'\n'))
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
