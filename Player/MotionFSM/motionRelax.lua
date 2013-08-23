local Body = require'Body'
local timeout = 1.0
local t_entry, t_update

local state = {}
state._NAME = 'motionRelax'

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
  if t - t_entry > timeout then return'timeout' end

end

function state.exit()
  print(state._NAME..' Exit' ) 
end

return state