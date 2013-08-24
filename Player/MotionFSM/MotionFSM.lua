-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local motionWalk   = require(Config.dev.walk)
local motionIdle  = require'motionIdle'
local motionStance = require'motionStance'
local motionFall   = require'motionFall'
--[[
local motionSit    = require'motionSit'
--]]

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( motionIdle, motionStance, motionWalk, motionFall )

-- Setup the transitions for this FSM
sm:set_transition(motionIdle, 'stand',   motionStance )
sm:set_transition(motionIdle, 'walk',    motionWalk )
sm:set_transition(motionIdle, 'fall',    motionFall )
sm:set_transition(motionIdle, 'timeout', motionIdle )
--
sm:set_transition(motionStance, 'done',    motionIdle)
sm:set_transition(motionStance, 'fall',    motionFall)
sm:set_transition(motionStance, 'timeout', motionStance)
--sm:set_transition(motionStance, 'walk', motionWalk)
--sm:set_transition(motionStance, 'sit',  motionSit)
--
--sm:set_transition(motionWalk, 'sit',    motionSit)
sm:set_transition(motionWalk, 'stand', motionStance)
sm:set_transition(motionWalk, 'fall', motionFall)
--[[
sm:set_transition(motionSit, 'done',    motionIdle)
sm:set_transition(motionSit, 'standup', motionStance)
--]]

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

return obj