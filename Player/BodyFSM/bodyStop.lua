local state = {}
state._NAME = ...


local Body = require'Body'

local timeout = 10.0
local t_entry, t_update, t_exit
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
  if mcm.get_walk_ismoving()>0 then
    mcm.set_walk_stoprequest(1) --stop if we're walking
  end
  old_state = gcm.get_game_state()
end

function state.update()
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t  
  if gcm.get_game_state()==3 then
    if old_state~=3 then
      --Just into play, reset pose
      wcm.set_robot_reset_pose(1)  
    end
    if wcm.get_robot_timestarted()==0 then
      wcm.set_robot_timestarted(t)
    end
    if gcm.get_game_role()==0 then --goalie
      print("Goalie start!")
      return'goalie'
    else
      print("Attacker start!")
      return'play'
    end
  else
    wcm.set_robot_timestarted(0)    
  end
end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()  
end

return state
