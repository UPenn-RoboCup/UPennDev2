local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update, t_exit

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Initialize all other state machines
  motion_ch:send'uninit'
  head_ch:send'teleop'

end

function state.update()
  --  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --TODO: Check whether all FSMs have done initialzing 

  hcm.set_motion_headangle({0,0})--zero headangle

  local body_init = mcm.get_status_body_init()
  if body_init==1  then return "done" end
end

function state.exit()
  --Release velocity limits for all the servos here
  hcm.set_step_stepcount(1)
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
