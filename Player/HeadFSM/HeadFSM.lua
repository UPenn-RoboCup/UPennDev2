-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local headIdle = require'headIdle'
local headTiltScan = require'headTiltScan'
local headTeleop = require'headTeleop'
local headCenter = require'headCenter'

local headScan = require'headScan'
local headTrack = require'headTrack'
local headLookGoal = require'headLookGoal'
local headSweep = require'headSweep'

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new(headIdle)
sm:add_state(headTiltScan)
sm:add_state(headTeleop)
sm:add_state(headCenter)

sm:add_state(headScan)
sm:add_state(headTrack)
sm:add_state(headLookGoal)
sm:add_state(headSweep)


-- Setup the transistions for this FSM
sm:set_transition(headIdle, 'teleop', headTeleop)
sm:set_transition(headIdle, 'center', headCenter)
--
sm:set_transition(headTeleop, 'reset', headIdle)
sm:set_transition(headTeleop, 'center', headCenter)
--
sm:set_transition(headCenter, 'done', headIdle)
--
sm:set_transition(headIdle, 'scan', headScan)
sm:set_transition(headScan, 'ballfound', headTrack)
sm:set_transition(headScan, 'center', headCenter)
--
sm:set_transition(headTrack, 'balllost', headScan)
sm:set_transition(headTrack, 'center', headCenter)
sm:set_transition(headTrack, 'timeout', headLookGoal)
sm:set_transition(headTrack, 'sweep', headSweep)
--
sm:set_transition(headLookGoal, 'timeout', headTrack)
sm:set_transition(headLookGoal, 'lost', headSweep)
--
sm:set_transition(headSweep, 'done', headTrack)

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
