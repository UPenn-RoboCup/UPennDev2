local state = {}
state._NAME = ...
local get_time = require'Body'.get_time
local t_entry, t_update, t_exit
local timeout = 10.0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = get_time()
  t_update = t_entry
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
