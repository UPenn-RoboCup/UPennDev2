local Body = require'Body'
local t_entry, t_update
local state = {}
state._NAME = ...

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  Body.set_head_command_position( {0,-0.25} )
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  return 'done'
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
