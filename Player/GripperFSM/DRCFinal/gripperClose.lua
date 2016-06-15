local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update

function state.entry()
  print(state._NAME..' Entry')
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Write the torque
	Body.set_lgrip_command_torque{5,5,20}
	

-- right gripper : nx motor
  --Body.set_rgrip_command_torque{5,5,20}

  Grip_hold = {0,0,0}
  Grip_open = {0.496719, 0.458369, 0.50132}

  print("Gripper is closed")
  Body.set_rgrip_torque_enable(1)
  Body.set_rgrip_command_position(Grip_hold)


end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

end

function state.exit()
  print(state._NAME..' Exit')
	-- Save our settings
end

return state
