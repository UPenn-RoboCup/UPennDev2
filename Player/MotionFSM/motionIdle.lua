local state = {}
state._NAME = ...

local Body = require'Body'
local timeout = 10.0
local t_entry, t_update

function state.entry()
  print(state._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
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
end

function state.exit()
  --Continuously read current leg position and write them to command position 
  Body.request_lleg_position()
  Body.request_rleg_position()
  Body.request_waist_position()

  local qLLeg, updatedL = Body.get_lleg_position()
  local qRLeg, updatedR = Body.get_rleg_position()
  local qWaist, updatedW = Body.get_waist_position()

  if updatedL then 
    Body.set_lleg_position(qLLeg)
  end
  if updatedR then 
    Body.set_rleg_position(qRLeg)
  end
  if updatedW then 
    Body.set_waist_position(qWaist)
  end

  print(state._NAME..' Exit' ) 
end

return state