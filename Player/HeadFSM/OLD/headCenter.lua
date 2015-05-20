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
  --TODO: need to compensate the torso pose
  Body.set_head_command_position( {0,-Config.walk.bodyTilt-0.05} )
  --Body.set_head_command_position( {0,-Config.walk.bodyTilt} )
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  print(unpack(Body.get_head_command_position()))
  return 'done'
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
