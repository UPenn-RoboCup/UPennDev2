local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update

local old_role = nil


function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  gcm.set_game_state(2)
  old_role = nil
  body_ch:send'stop'
  head_ch:send'teleop'
  mcm.set_walk_stoprequest(1)

end

function state.update()
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

--WE NEED TO RESET BALL POS AT INITIAL
--OTHERWISE we think ball jumps all the time!!
  wcm.set_ball_observed(0)


  local role = gcm.get_game_role()

  if role~=old_role then
    if role==1 then --attacker
      print( util.color('Attacker Set','red') )  
      print( util.color('Attacker Set','red') )  
      print( util.color('Attacker Set','red') )  
    elseif role==0 then
      print( util.color('GOALIE Set','blue') )
      print( util.color('GOALIE Set','blue') )
      print( util.color('GOALIE Set','blue') )
    elseif role==2 then
      print( 'Testing' )
    elseif role==3 then
      print( util.color('DEMO Initial','green') )
      print( util.color('DEMO Initial','green') )
      print( util.color('DEMO Initial','green') )
    end
    old_role=role
  end
end

function state.exit()
  print(state._NAME..' Exit' ) 
end

return state
