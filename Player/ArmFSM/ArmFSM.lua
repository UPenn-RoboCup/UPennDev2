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

--Default pose for walking (arm slightly spread)
local armPose1 = require'armPose1'

--Default pose for kneeling (arm higher not to touch the ground)
local armPose2 = require'armPose2'

-- Move arm to pose 1 (from Idle, Pose2, doorgrip)
local armChangetoPose1 = require'armChangetoPose1'

-- Move arm to pose 2 (from teleop, wheelrelease)
local armChangetoPose2 = require'armChangetoPose2'

-- Direct teleop override
local armTeleop = require'armTeleop'

local armIKTest = require'armIKTest'

local armRocky = require'armRocky'

-- Wheel specific states
local armWheelGrip = require'armWheelGrip'
local armWheelTurn = require'armWheelTurn'
local armWheelRelease = require'armWheelRelease'
local armWheelTurnValve = require'armWheelTurnValve'

-- Door specific states
local armDoorGrip = require'armDoorGrip'
local armDoorRelease = require'armDoorRelease'

-- Tool specific states
local armToolGrip = require'armToolGrip'
local armToolHold = require'armToolHold'
local armToolChop = require'armToolChop'

-- Small circular valve turning (one hand turning)
local armSmallValveGrip = require'armSmallValveGrip'

-- Large circular valve turning (two hand turning)
local armLargeValveGrip = require'armLargeValveGrip'

-- bar valve turning
local armBarValveGrip = require'armBarValveGrip'




local armDebrisGrip = require'armDebrisGrip'



local armSupportDoor = require'armSupportDoor'





local sm = fsm.new(armIdle);
sm:add_state(armPose1)
sm:add_state(armPose2)
sm:add_state(armChangetoPose1)
sm:add_state(armChangetoPose2)
sm:add_state(armTeleop)
sm:add_state(armWheelGrip)
sm:add_state(armWheelTurn)
sm:add_state(armWheelTurnValve)
sm:add_state(armWheelRelease)
sm:add_state(armDoorGrip)
sm:add_state(armDoorRelease)

sm:add_state(armToolGrip)
sm:add_state(armToolHold)
sm:add_state(armToolChop)

sm:add_state(armSmallValveGrip)
sm:add_state(armLargeValveGrip)
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
sm:set_transition(armIdle, 'init', armChangetoPose1)

sm:set_transition(armChangetoPose1, 'done', armPose1)
sm:set_transition(armChangetoPose2, 'done', armPose2)

--sm:set_transition(armPose1, 'ready', armChangetoPose2)
sm:set_transition(armPose1, 'teleop', armTeleop)
--sm:set_transition(armPose1, 'teleop', armIKTest)




sm:set_transition(armPose1, 'rocky', armRocky)
sm:set_transition(armPose1, 'doorgrab', armDoorGrip)
sm:set_transition(armPose1, 'toolgrab', armToolGrip)
sm:set_transition(armPose1, 'wheelgrab', armWheelGrip)


sm:set_transition(armPose1, 'smallvalvegrab', armSmallValveGrip)
sm:set_transition(armPose1, 'largevalvegrab', armLargeValveGrip)
sm:set_transition(armPose1, 'barvalvegrab', armBarValveGrip)


sm:set_transition(armSmallValveGrip, 'done', armChangetoPose1)
sm:set_transition(armLargeValveGrip, 'done', armChangetoPose1)
sm:set_transition(armBarValveGrip, 'done', armChangetoPose1)


sm:set_transition(armRocky, 'reset', armChangetoPose1)

--sm:set_transition(armPose2, 'toolgrab', armToolGrip)


sm:set_transition(armPose2, 'debrisgrab', armDebrisGrip)


--sm:set_transition(armPose2, 'doorgrab', armDoorGrip)

--sm:set_transition(armWheelGrip, 'done', armWheelTurn)
sm:set_transition(armWheelGrip, 'done', armWheelTurnValve)
sm:set_transition(armWheelGrip, 'reset', armChangetoPose1)
sm:set_transition(armWheelTurn, 'reset', armWheelRelease)


sm:set_transition(armWheelTurnValve, 'done', armWheelRelease)
sm:set_transition(armWheelTurnValve, 'reset', armWheelRelease)
sm:set_transition(armWheelRelease, 'done', armChangetoPose1)


--TODO: should use IK to get back?
--sm:set_transition(armToolGrip, 'reset', armChangetoPose2)

sm:set_transition(armToolGrip, 'reset', armChangetoPose1)
sm:set_transition(armToolGrip, 'done', armToolHold)
sm:set_transition(armToolHold, 'toolgrab', armToolChop)
sm:set_transition(armToolChop, 'done', armToolHold)


-- The initial arm pose is great for door gripping, 
-- and should be the reset position
sm:set_transition(armDoorGrip, 'reset', armDoorRelease)
sm:set_transition(armDoorGrip, 'planfail', armPose1)
sm:set_transition(armDoorGrip, 'done', armChangetoPose1)
sm:set_transition(armDoorRelease, 'done', armChangetoPose1)

-- TODO: This may not be the best
-- We may wish to give ready and init
-- TODO: make epi transitions for reset
sm:set_transition(armTeleop, 'reset', armChangetoPose1)

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