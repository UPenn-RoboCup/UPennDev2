local state = {}
state._NAME = ...
local util   = require'util'


local Body = require'Body'
local robocupplanner = require'robocupplanner'
local timeout = 10.0
local t_entry, t_update, t_exit, t_plan
require'gcm'

--Tele-op state for testing various stuff
--Don't do anything until commanded
local old_state

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry    

end

function state.update()
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()  
end

return state
