-- Config guides special situations
local Config = require'Config'
local util = require'util'
-- Use the fsm module
local fsm = require'fsm'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber('fsm_motion',true)

-- Require the needed states
local headIdle = require'headIdle'
local headTiltScan = require'headTiltScan'

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( headIdle, headTiltScan )

-- Setup the transistions for this FSM
sm:set_transition(headIdle, 'tiltscan', headTiltScan)
--
sm:set_transition(headTiltScan, 'done', headIdle)

local obj = {}
obj._NAME = 'Head'
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