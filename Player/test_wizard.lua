dofile'include.lua'

IS_TESTING = true --skip body update

global_clock = 0


local Body = require'Body'
local util = require'util'
local mp   = require'msgpack'
local signal = require'signal'
local simple_ipc = require'simple_ipc'
local udp        = require'udp'
local vector     = require'vector'

local state_pub_ch = simple_ipc.new_publisher(Config.net.state)
local pulse_ch = simple_ipc.new_publisher'pulse'

require'gcm'
require'wcm'
require'jcm'
require'mcm'
require'hcm'




--SJ: This removes the output buffer 
io.stdout:setvbuf("no")

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
  local ret = state_pub_ch:send( mp.pack(status) )
end

local all_joint_read = vector.ones(#jcm.get_read_position())
local function set_read_all()
  jcm.set_read_position(all_joint_read)
end
while true do
  local t = Body.get_time()
  
  -- Update each state machine
  for _,my_fsm in pairs(state_machines) do
    local event = my_fsm.update()
  end

  -- Broadcast state changes
  if needs_broadcast then
    needs_broadcast = false
    -- Broadcast over UDP/TCP/IPC
    local ret = state_pub_ch:send( mp.pack(status) )
  end
  
  -- Always read from all the motors
  set_read_all()
  
  -- Update the body (mostly needed for webots)
	Body.update()
  
  -- Send the tick tock at the end of a cycle
  pulse_ch:send(tostring(t))
  
  --Proceed simulated clock


  --We are only testing the planning, so the tstep can be huge
  global_clock = global_clock + 0.2 

  hcm.set_state_proceed(1) --Fast forward the state transition!
  local success = hcm.get_state_success()

  if success == 0 then --still moving

  elseif success ==1 then --success
    local tstartactual = hcm.get_state_tstartactual()
    local tstartrobot = hcm.get_state_tstartrobot()

    local tpassed_actual = unix.time() - tstartactual
    local tpassed_robot = Body.get_time() - tstartrobot
    
    print(string.format("SUCCESS, %d ms passed, robot time %.1f passed",
      tpassed_actual*1000, tpassed_robot
      ))
    state_machines['ArmFSM'].sm:add_event('forcereset')
    hcm.set_state_success(0)
  else --failed somewhere
    print("FAIL")
    state_machines['ArmFSM'].sm:add_event('forcereset')
    hcm.set_state_success(0)
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
