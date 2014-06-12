local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update, qH

function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry

  -- Torque OFF the motors
  Body.set_head_torque_enable(0)
  wcm.set_ball_t(Body.get_time()-10.0) --to fix bad shm value
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

end

function state.exit()
  print(state._NAME..' Exit' )

  -- Torque on the motor
  Body.set_head_torque_enable({1, 1})
end

return state
