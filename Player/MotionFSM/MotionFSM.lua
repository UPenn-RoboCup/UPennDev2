-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local motionIdle   = require'motionIdle'
local motionFall   = require'motionFall'
local motionWalk   = require(Config.dev.walk)
local motionStance = require'motionStance'
local motionStep   = require'motionStep'
local motionStepPreview   = require'motionStepPreview'

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( motionIdle, motionStance, motionFall )
sm:add_state(motionWalk)
sm:add_state(motionStep)
sm:add_state(motionStepPreview)

-- Setup the transitions for this FSM
sm:set_transition(motionIdle, 'stand', motionStance )
--
sm:set_transition(motionStance, 'done', motionIdle, function(exit_val)
  -- Stance exit value should be some foot positions...
  -- These foot positions *should* be in shared memory though
  -- Initially, the walk event is disallowed, since we do not know our configuration
  sm:set_transition(motionIdle, 'walk', motionWalk )
  sm:set_transition(motionIdle, 'step', motionStep )
  sm:set_transition(motionIdle, 'preview', motionStepPreview )
end)
--
sm:set_transition(motionWalk, 'stand', motionStance, function(exit_val)
  -- Walk exit value should be some foot positions...
  -- These foot positions *should* be in shared memory though
end)

sm:set_transition(motionStep, 'done', motionIdle )
--
sm:set_transition(motionStepPreview, 'done', motionIdle )





--------------------------
-- Setup the FSM object --
--------------------------
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
  -- TODO: If falling, maybe just call that update function?
  sm:update()
end
obj.exit = function()
  sm:exit()
end

obj.sm = sm

return obj