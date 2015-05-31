local state = {}
state._NAME = ...
require'mcm'
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!

local Body = require'Body'
local timeout = 10.0
local t_entry, t_update

-- Running estimate of where the legs are
local qRLeg0

function state.entry()
  print(state._NAME..' Entry' ) 
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Torque OFF the motors
  Body.set_lleg_torque_enable({1,0,0, 0,0,0})
  Body.set_rleg_torque_enable({1,0,0, 0,0,0})
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
--  print(state._NAME..' Update')
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
end

function state.exit()
  print(state._NAME..' Exit' ) 

  Body.set_lleg_torque_enable(1)
  Body.set_rleg_torque_enable(1)
end

return state
