--------------------------------
-- Humanoid arm epi state machine
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------

-- fsm module
local fsm = require'fsm'

-- Require the needed states
local armIdle = require'armIdle'
local armInit = require'armInit'
-- From Init toward Ready
local armInitReady = require'armInitReady'
local armReady = require'armReady'
-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new(armIdle,armInit,armInitReady,armReady)
--

-- Direct teleop override
local armTeleop = require'armTeleop'
sm:add_state(armTeleop)

-- Wheel specific states
local armWheelGrip = require'armWheelGrip'
local armWheelTurn = require'armWheelTurn'
sm:add_state(armWheelGrip)
sm:add_state(armWheelTurn)

-- Door specific states
local armDoorGrip = require'armDoorGrip'
--local armDoorTurn = require'armDoorTurn'
sm:add_state(armDoorGrip)
--sm:add_state(armDoorTurn)

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
--
sm:set_transition(armInit, 'done', armIdle, function()
  -- When we are in the idle (init) state,
  -- we are allowed to make some transitions
  sm:set_transition(armIdle, 'ready', armInitReady)
  -- The initial position is great for grabbing the door
  sm:set_transition(armIdle, 'doorgrab', armDoorGrip)
end)
--
function armInitReady.epi.update()
  -- Set the direction epi-state when armInitReady is done
  if not armInitReady.epi.reverse then armInitReady.epi.reverse = false end
  armInitReady.epi.reverse = not armInitReady.epi.reverse
  if armInitReady.epi.reverse then
    sm:set_transition(armInitReady, 'done', armInit)
    -- Resetting armInitReady now means to go back to armReady
    sm:set_transition(armInitReady, 'reset', armReady)
  else
    sm:set_transition(armInitReady, 'done', armReady)
    -- Resetting armInitReady now means to go back to armReady
    sm:set_transition(armInitReady, 'reset', armInit)
  end
end
sm:set_transition(armInitReady, 'reset', armInit)
-- Stateful transitions
sm:set_transition(armInitReady, 'done', armReady, armInitReady.epi.update)
--
sm:set_transition(armReady, 'reset', armInitReady)
sm:set_transition(armReady, 'done', armIdle, function()
  -- When we are in the idle (ready) state,
  -- we are allowed to make some transitions
  sm:set_transition(armIdle, 'reset', armInitReady)
  sm:set_transition(armIdle, 'ready', armReady)
  -- Manipulation ability!
  -- TODO: How to remove (prune) this functionality when back in init?
  -- The ready position is great for teleop and wheel grabbing
  -- It's a bit awkward for the door opening
  sm:set_transition(armIdle, 'teleop', armTeleop)
  sm:set_transition(armIdle, 'wheelgrab', armWheelGrip)
  -- Cannot do anymore
  sm:remove_transition(armIdle, 'doorgrab', armDoorGrip)
end)
--
sm:set_transition(armWheelGrip, 'reset', armReady)
sm:set_transition(armWheelGrip, 'done', armWheelTurn)
--
sm:set_transition(armWheelTurn, 'reset', armReady)
--
-- The initial arm pose is great for door gripping, 
-- and should be the reset position
sm:set_transition(armDoorGrip, 'reset', armInit)
sm:set_transition(armDoorGrip, 'done',  armTeleop)
-- TODO: This may not be the best
-- We may wish to give ready and init
-- TODO: make epi transitions for reset
sm:set_transition(armTeleop, 'reset', armInit)

-- Setup the FSM object
local obj = {}
local util = require'util'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber(...,true)
obj._NAME = ...
obj.entry = function()
  sm:entry()
end
obj.update = function()
  -- Check for out of process events in non-blocking
  local event, has_more = evts:receive(true)
  if event then
    print( util.color(obj._NAME..' Event:','green'),event)
    sm:add_event(event)
  end
  sm:update()
end
obj.exit = function()
  sm:exit()
end

obj.sm = sm

return obj