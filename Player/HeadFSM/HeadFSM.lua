-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local headIdle = require'headIdle'
local headTiltScan = require'headTiltScan'
local headTeleop = require'headTeleop'
local headCenter = require'headCenter'


local headFireScan = require'headFireScan'
local headFireTrack = require'headFireTrack'

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new(headIdle)
sm:add_state(headTiltScan)
sm:add_state(headTeleop)
sm:add_state(headCenter)
sm:add_state(headFireScan)
sm:add_state(headFireTrack)

-- Setup the transistions for this FSM
--sm:set_transition(headIdle, 'tiltscan', headTiltScan)
sm:set_transition(headIdle, 'teleop', headTeleop)
sm:set_transition(headIdle, 'center', headCenter)


--
--sm:set_transition(headTiltScan, 'tiltscan', headTiltScan)
--sm:set_transition(headTiltScan, 'reset', headIdle)
--sm:set_transition(headTiltScan, 'center', headCenter)
--
sm:set_transition(headTeleop, 'reset', headIdle)
sm:set_transition(headTeleop, 'center', headCenter)
--
sm:set_transition(headCenter, 'done', headIdle)


sm:set_transition(headTeleop, 'scan', headFireScan)
sm:set_transition(headIdle, 'scan', headFireScan)

sm:set_transition(headFireScan, 'track', headFireTrack)
sm:set_transition(headFireTrack, 'lost', headFireScan)

--------------------------
-- Setup the FSM object --
--------------------------
local obj = {}
obj._NAME = ...
local util = require'util'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber(...,true)
local debug_str = util.color(obj._NAME..' Event:','green')
local special_evts = special_evts or {}
obj.entry = function()
  sm:entry()
end
obj.update = function()
  -- Check for out of process events in non-blocking fashion
  local event, has_more = evts:receive(true)
  if event then
    local debug_tbl = {debug_str,event}
    if has_more then
      extra = evts:receive(true)
      table.insert(debug_tbl,'Extra')
    end
    local special = special_evts[event]
    if special then
      special(extra)
      table.insert(debug_tbl,'Special!')
    end
    print(table.concat(debug_tbl,' '))
    sm:add_event(event)
  end
  sm:update()
end
obj.exit = function()
  sm:exit()
end

obj.sm = sm

return obj