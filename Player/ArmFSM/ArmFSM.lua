-- fsm module
local fsm = require'fsm'

-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber('fsm_arm',true)

-- Require the needed states
local armIdle   = require('armIdle')
local armInit   = require('armInit')
local armReset  = require('armReset')
local armReady  = require('armReady')
local armTeleop = require('armTeleop')

-- Wheel specific states
local armWheelGrip = require('armWheelGrip')
local armWheelTurn = require('armWheelTurn')
local armWheelRelease = require('armWheelRelease')

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new(armIdle)
sm:add_state(armInit)
sm:add_state(armReady)
sm:add_state(armReset)
sm:add_state(armWheelGrip)
sm:add_state(armWheelTurn)
sm:add_state(armWheelRelease)
sm:add_state(armTeleop)

-- Setup the transitions for this FSM
sm:set_transition(armIdle, 'init', armInit)
--
sm:set_transition(armInit, 'done', armReady)
--
sm:set_transition(armReset, 'done', armIdle)
--
sm:set_transition(armReady, 'wheelgrab', armWheelGrip)
sm:set_transition(armReady, 'reset', armReset)
--
sm:set_transition(armWheelGrip, 'reset', armWheelRelease)
sm:set_transition(armWheelGrip, 'done', armWheelTurn)
sm:set_transition(armWheelGrip, 'stop', armReady)
sm:set_transition(armWheelGrip, 'reset', armWheelRelease)
--
sm:set_transition(armWheelTurn, 'reset', armWheelRelease)
sm:set_transition(armWheelTurn, 'stop', armWheelRelease)
--
sm:set_transition(armWheelRelease, 'done', armReady)

-- Setup the FSM object for use in the main routine
local obj = {}
obj._NAME = 'Arm'
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