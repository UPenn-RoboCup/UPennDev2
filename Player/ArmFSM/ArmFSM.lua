--------------------------------
-- Humanoid arm epi state machine
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------

-- fsm module
local fsm = require'fsm'

--SJ: Simplified the arm FSMs

-- Require the needed states

--Default state, arm can be any position
local armIdle = require'armIdle'

-- Move arm to pose 1 from Idle
local armInit = require'armInit'

--Default pose for walking (arm slightly spread)
local armPose1 = require'armPose1'


-- Direct teleop override
local armTeleop = require'armTeleop'

local armIKTest = require'armIKTest'

local armRocky = require'armRocky'


-- Door specific states
local armDoorGrip = require'armDoorGrip'

-- Tool specific states
local armToolGrip = require'armToolGrip'
local armToolHold = require'armToolHold'
local armToolChop = require'armToolChop'

-- Small circular valve turning (one hand turning)
local armSmallValveGrip = require'armSmallValveGrip'

-- Large circular valve turning (one hand turning)
local armLargeValveGrip = require'armLargeValveGrip'

-- Large circular valve turning (two hand turning)
local armLargeValveGripTwohand = require'armLargeValveGripTwohand'

-- bar valve turning
local armBarValveGrip = require'armBarValveGrip'




local armDebrisGrip = require'armDebrisGrip'



local armSupportDoor = require'armSupportDoor'

local armForceReset = require'armForceReset'



local sm = fsm.new(armIdle);
sm:add_state(armInit)
sm:add_state(armForceReset)
sm:add_state(armPose1)
sm:add_state(armTeleop)
sm:add_state(armDoorGrip)

sm:add_state(armToolGrip)
sm:add_state(armToolHold)
sm:add_state(armToolChop)

sm:add_state(armSmallValveGrip)
sm:add_state(armLargeValveGrip)
sm:add_state(armLargeValveGripTwohand)
sm:add_state(armBarValveGrip)
sm:add_state(armDebrisGrip)

sm:add_state(armRocky)

sm:add_state(armSupportDoor)

sm:add_state(armIKTest)
----------
-- Event types
----------
-- 'reset': This exists for relay states, like armInitReady
--          where you go back to the start end of the relay
--          before finishing the relay.  Human guided (or robot predicted)
-- 'done':  Attained that state.  Epi-callbacks are preferable.
-- Fully autonomous, since the human can estimate the state?

-- Setup the transitions for this FSM
sm:set_transition(armIdle, 'init', armInit)
sm:set_transition(armInit, 'done', armPose1)
--sm:set_transition(armPose1, 'teleop', armTeleop)
sm:set_transition(armPose1, 'teleop', armIKTest)
sm:set_transition(armIKTest, 'teleop', armTeleop)



sm:set_transition(armPose1, 'doorgrab', armDoorGrip)
sm:set_transition(armPose1, 'toolgrab', armToolGrip)
sm:set_transition(armPose1, 'debrisgrab', armDebrisGrip)
sm:set_transition(armPose1, 'smallvalvegrab', armSmallValveGrip)
sm:set_transition(armPose1, 'largevalvegrab', armLargeValveGrip)
sm:set_transition(armPose1, 'largevalvetwograb', armLargeValveGripTwohand)
sm:set_transition(armPose1, 'barvalvegrab', armBarValveGrip)


sm:set_transition(armSmallValveGrip, 'done', armPose1)
sm:set_transition(armLargeValveGrip, 'done', armPose1)
sm:set_transition(armLargeValveGripTwohand, 'done', armPose1)
sm:set_transition(armBarValveGrip, 'done', armPose1)

sm:set_transition(armToolGrip, 'done', armPose1)
sm:set_transition(armToolGrip, 'hold', armToolHold)
sm:set_transition(armToolHold, 'toolgrab', armToolChop)
sm:set_transition(armToolChop, 'done', armToolHold)




sm:set_transition(armDoorGrip, 'done', armPose1)
sm:set_transition(armDebrisGrip, 'done', armPose1)
sm:set_transition(armTeleop, 'done', armPose1)



--Force reset states is used for offline testing only
sm:set_transition(armSmallValveGrip, 'forcereset', armForceReset)
sm:set_transition(armLargeValveGrip, 'forcereset', armForceReset)
sm:set_transition(armBarValveGrip, 'forcereset', armForceReset)
sm:set_transition(armToolGrip, 'forcereset', armForceReset)
sm:set_transition(armToolHold, 'forcereset', armForceReset)
sm:set_transition(armDoorGrip, 'forcereset', armForceReset)
sm:set_transition(armDebrisGrip, 'forcereset', armForceReset)

sm:set_transition(armForceReset, 'done', armPose1)









--depreciated
sm:set_transition(armPose1, 'rocky', armRocky)
sm:set_transition(armRocky, 'rocky', armInit) 

--------------------------
-- Setup the FSM object --
--------------------------
local obj = {}
obj._NAME = ...
local util = require'util'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber(...,true)
local debug_str = util.color(obj._NAME..' Event:','green')
local special_evts = special_evts or {}
obj.entry = function()
  sm:entry()
end
obj.update = function()
  -- Check for out of process events in non-blocking fashion
  local event, has_more = evts:receive(true)
  if event then
    local debug_tbl = {debug_str,event}
    if has_more then
      extra = evts:receive(true)
      table.insert(debug_tbl,'Extra')
    end
    local special = special_evts[event]
    if special then
      special(extra)
      table.insert(debug_tbl,'Special!')
    end
    print(table.concat(debug_tbl,' '))
    sm:add_event(event)
  end
  sm:update()
end
obj.exit = function()
  sm:exit()
end

obj.sm = sm

return obj