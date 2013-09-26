-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states

local motionIdle   = require'motionIdle' --Initial state, all legs torqued off
local motionInit   = require'motionInit' --Torque on legs and go to initial leg position
local motionStance = require'motionStance' --Robots stands still, balancing itself
local motionWalk   = require(Config.dev.walk) --Reactive walking
local motionStep   = require'motionStep'   --Stationary stepping
local motionStepPreview   = require'motionStepPreview' --ZMP preview stepping

--Our robot never fall!
--local motionFall   = require'motionFall' 

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( motionIdle, motionInit, motionStance)
--, motionFall )
sm:add_state(motionWalk)
sm:add_state(motionStep)
sm:add_state(motionStepPreview)


--[[
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
--]]

sm:set_transition(motionIdle, 'stand', motionInit)
sm:set_transition(motionInit, 'done', motionStance)

sm:set_transition(motionStance, 'walk', motionWalk)
sm:set_transition(motionStance, 'step', motionStep)
sm:set_transition(motionStance, 'preview', motionStepPreview)

--Walk stop should be handled elsewise
--As it can stop walking mid-step
sm:set_transition(motionWalk, 'stand', motionStance)
sm:set_transition(motionStep, 'done', motionStance)
sm:set_transition(motionStepPreview, 'done', motionStance)

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