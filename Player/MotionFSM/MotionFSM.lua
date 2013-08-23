-- Config guides special situations
local Config = require'Config'
local util = require'util'
-- Use the fsm module
local fsm = require'fsm'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber('fsm_motion',true)

-- Require the needed states
local motionWalk   = require(Config.dev.walk)
local motionRelax  = require'motionRelax'
local motionStance = require'motionStance'
local motionFall   = require'motionFall'
--[[
local motionSit    = require'motionSit'
--]]

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( motionRelax, motionStance, motionWalk, motionFall )

-- Setup the transitions for this FSM
sm:set_transition(motionRelax, 'stand',   motionStance )
sm:set_transition(motionRelax, 'walk',    motionWalk )
sm:set_transition(motionRelax, 'fall',    motionFall )
sm:set_transition(motionRelax, 'timeout', motionRelax )
--
sm:set_transition(motionStance, 'done',    motionRelax)
sm:set_transition(motionStance, 'fall',    motionFall)
sm:set_transition(motionStance, 'timeout', motionStance)
--sm:set_transition(motionStance, 'walk', motionWalk)
--sm:set_transition(motionStance, 'sit',  motionSit)
--
--sm:set_transition(motionWalk, 'sit',    motionSit)
sm:set_transition(motionWalk, 'stand', motionStance)
sm:set_transition(motionWalk, 'fall', motionFall)
--[[
sm:set_transition(motionSit, 'done',    motionRelax)
sm:set_transition(motionSit, 'standup', motionStance)
--]]

-- Setup the FSM object for use in the main routine
local obj = {}
obj._NAME = 'Motion'
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

return obj