local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0
require'gcm'
require'wcm'

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  gcm.set_game_state(5)-- Always start as the idle state  
  gcm.set_game_role(2) -- Always start as the tester role
  wcm.set_robot_traj_num(0)
end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_updae = t
  print("Default Folder")
  local gamestate = gcm.get_game_state()
  if gamestate~=5 then return 'init' end --5 is idle state 

  if Config.auto_state_advance and t-t_entry>1 then
    if gamestate==5 then 
      print"body init!!!!"
      gcm.set_game_state(0)
      return 'init' 
    end
  end
end

function state.exit()
  print("Default Folder")
  print(state._NAME..' Exit' )
end

return state
