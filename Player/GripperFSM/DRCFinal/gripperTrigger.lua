
local state = {}
state._NAME = ...

local Body = require'Body'

local t_entry, t_update

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	-- gripper, trigger, extra
	--Body.set_lgrip_command_torque{20,40,20}
	Body.set_rgrip_command_torque{20,40,20}

	-- TODO: Add a check to see if the trigger worked or not

end

function state.exit()
  print(state._NAME..' Exit')
end

return state
