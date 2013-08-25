-- Config guides special situations
local Config = require'Config'
-- fsm module
local fsm = require'fsm'

-- Require the needed states
local armIdle = require'armIdle'
local armInit = require'armInit'
local armPreReady  = require'armInitReady' -- From Init toward Ready
local armPostReady = require'armInitReady' -- From Ready toward Init
local armReady = require'armReady'
--[[
local armTeleop = require('armTeleop')
-- Wheel specific states
local armWheelGrip = require('armWheelGrip')
local armWheelTurn = require('armWheelTurn')
local armWheelRelease = require('armWheelRelease')
--]]

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new(armIdle,armInit,armPreReady,armReady,armPostReady)
--[[
sm:add_state(armWheelGrip)
sm:add_state(armWheelTurn)
sm:add_state(armWheelRelease)
sm:add_state(armTeleop)
--]]

-- Setup the transitions for this FSM
sm:set_transition(armIdle, 'init', armInit)
--
sm:set_transition(armInit, 'ready', armPreReady)
--
sm:set_transition(armPreReady, 'reset', armInit)
sm:set_transition(armPreReady, 'done', armReady)
--
sm:set_transition(armPostReady, 'done', armInit)
sm:set_transition(armPostReady, 'reset', armReady)
--
sm:set_transition(armReady, 'init', armPostReady)
--sm:set_transition(armReady, 'wheelgrab', armWheelGrip)
--[[
sm:set_transition(armWheelGrip, 'reset', armWheelRelease)
sm:set_transition(armWheelGrip, 'done', armWheelTurn)
sm:set_transition(armWheelGrip, 'stop', armReady)
sm:set_transition(armWheelGrip, 'reset', armWheelRelease)
--
sm:set_transition(armWheelTurn, 'reset', armWheelRelease)
sm:set_transition(armWheelTurn, 'stop', armWheelRelease)
--
sm:set_transition(armWheelRelease, 'done', armReady)
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

obj.sm = sm

return obj