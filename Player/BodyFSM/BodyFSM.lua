--------------------------------
-- Humanoid arm state machine
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local bodyIdle     = require'bodyIdle'
local bodyInit     = require'bodyInit'

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new(bodyIdle,bodyInit)

local bodyTeleop = require'bodyTeleop'
sm:add_state(bodyTeleop)

local bodyFollow = require'bodyFollow'
sm:add_state(bodyFollow)

local bodyNavigate = require'bodyNavigate'
sm:add_state(bodyNavigate)

-- Setup the transistions for this FSM
--
sm:set_transition(bodyIdle, 'init', bodyInit)
--
sm:set_transition(bodyInit, 'follow', bodyFollow)
--
sm:set_transition( bodyInit,   'teleop', bodyTeleop )
sm:set_transition( bodyTeleop, 'reset',  bodyInit   )
sm:set_transition( bodyTeleop, 'follow', bodyFollow )


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