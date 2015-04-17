
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
  Body.set_lgrip_torque_enable(0)
	Body.set_rgrip_torque_enable(0)
	-- Try to ensure we are in the correct mode
	for i=1,2 do
		Body.set_lgrip_command_torque(0)
		Body.set_rgrip_command_torque(0)
		Body.set_lgrip_mode('torque')
		Body.set_rgrip_mode('torque')
		--if not IS_WEBOTS then unix.usleep(1e5) end
	end
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
  Body.set_lgrip_torque_enable(1)
	Body.set_rgrip_torque_enable(1)
	for i=1,2 do
		Body.set_lgrip_command_torque(0)
		Body.set_rgrip_command_torque(0)
		Body.set_lgrip_mode('torque')
		Body.set_rgrip_mode('torque')
		--if not IS_WEBOTS then unix.usleep(1e5) end
	end

	-- We are in torque mode to retain 0 torque
	hcm.set_teleop_lgrip_mode(0)
  hcm.set_teleop_rgrip_mode(0)

end

return state
