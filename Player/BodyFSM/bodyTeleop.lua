local Config = require'Config'
local Body   = require'Body'
--require'hcm'

local state = {}
state._NAME = 'bodyTeleop'

local t_entry, t_update, t_exit
local timeout = 10.0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- TODO: Fix this
  --Body.set_lwheel_velocity(0);
  --Body.set_rwheel_velocity(0);
end

function state.update()
  -- print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

  local wheelVel = hcm:get_control_wheel_velocity();
  Body.set_lwheel_velocity(wheelVel[1]);
  Body.set_rwheel_velocity(wheelVel[2]);
end

function state.exit()
  print(_NAME..' Exit' ) 
  Body.set_lwheel_velocity(0);
  Body.set_rwheel_velocity(0);
end

return state