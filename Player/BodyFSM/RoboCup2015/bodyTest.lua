local state = {}
state._NAME = ...
local util   = require'util'


local Body = require'Body'
local timeout = 10.0
local t_entry, t_update, t_exit, t_plan

--Tele-op state for testing various stuff
--Don't do anything until commanded
local old_state

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry


  motion_ch:send'hybridwalk'    
end


local tTest1, tTest2 = Config.walk.testT[1],Config.walk.testT[2]


function state.update()
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local t_passed = t-t_entry
  if t_passed<tTest1 then
    mcm.set_walk_vel({Config.walk.testVel2 or 0.15,0,0})
  else
    mcm.set_walk_vel({Config.walk.testVel1 or 0.08,0,0})
  end

  -- Save this at the last update time

  if t_passed>tTest2 then
    if mcm.get_walk_ismoving()>0 then      
      mcm.set_walk_stoprequest(1)
    else
      return "done"
    end
  end
end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()  
end

return state
