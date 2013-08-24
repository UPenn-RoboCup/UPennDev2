-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local bodyIdle = require'bodyIdle'
local bodyTeleop = require'bodyTeleop'
local bodyNavigate = require'bodyNavigate'

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new(bodyIdle)
sm:add_state(bodyTeleop)
sm:add_state(bodyNavigate)

-- Setup the transistions for this FSM
--
sm:set_transition(bodyIdle, 'teleop', bodyTeleop)
sm:set_transition(bodyIdle, 'navigate', bodyNavigate)
--
sm:set_transition(bodyTeleop, 'stop', bodyIdle)
sm:set_transition(bodyTeleop, 'navigate', bodyNavigate)
--
sm:set_transition(bodyNavigate, 'done', bodyIdle)
sm:set_transition(bodyNavigate, 'stop', bodyIdle)

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