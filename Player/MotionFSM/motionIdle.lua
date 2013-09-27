local state = {}
state._NAME = ...

-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!

local Body = require'Body'
local timeout = 10.0
local t_entry, t_update

-- Running estimate of where the legs are
local qLLeg, qRLeg, qWaist


function state.entry()
  print(state._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Torque OFF the motors
  Body.set_waist_torque_enable(0)
  Body.set_lleg_torque_enable(0)
  Body.set_rleg_torque_enable(0)

  -- Initialize our joint positions estimate
  qLLeg  = Body.get_lleg_command_position()
  qRLeg  = Body.get_rleg_command_position()
  qWaist = Body.get_waist_command_position()

  -- Request new readings
  Body.request_lleg_position()
  Body.request_rleg_position()
  Body.request_waist_position()

end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t - t_entry > timeout then return'timeout' end

  -- Grab our position if available
  local updatedL, updatedR, updatedW
  qL, updatedL = Body.get_lleg_position()
  qR, updatedR = Body.get_rleg_position()
  qW, updatedW = Body.get_waist_position()

  -- Set our global idea of where our joints are
  -- Continuously read current leg position and write them to command position 
  if updatedL then
    qLLeg = qL
    Body.request_lleg_position()
  end
  if updatedR then
    qRLeg = qR
    Body.request_rleg_position()
  end
  if updatedW then
    qWaist = qW
    Body.request_waist_position()
  end

end

function state.exit()
  print(state._NAME..' Exit' ) 

  -- Torque on the motors
  Body.set_waist_torque_enable(1)
  Body.set_lleg_torque_enable(1)
  Body.set_rleg_torque_enable(1)

  -- Wait 10 milliseconds for motors to turn on
  unix.usleep(1e4)

  -- Set the commanded position
  ----[[
  Body.set_lleg_command_position(qLLeg)
  Body.set_rleg_command_position(qRLeg)
  Body.set_waist_command_position(qWaist)
  --]]

end

return state