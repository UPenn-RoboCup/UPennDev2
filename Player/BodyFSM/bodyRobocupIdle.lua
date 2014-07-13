local state = {}
state._NAME = ...

local Body = require'Body'

local timeout = 10.0
local t_entry, t_update, t_exit
require'wcm'

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  --hcm.set_ball_approach(0)
  wcm.set_robot_traj_num(0)
end

function state.update()
  local role = gcm.get_game_role()
  if role==0 then return 'goalie' end
  
  --  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then
    return'timeout'
  end

  -- if we see ball right now and ball is far away start moving
  local ball_elapsed = t - wcm.get_ball_t()
  if ball_elapsed < 0.1 then --ball found
    return 'ballfound'
    --[[
    local ballx = wcm.get_ball_x()
    local bally = wcm.get_ball_y()
    local ballr = math.sqrt(ballx*ballx+bally*bally)
    local balla = math.atan2(bally,ballx)
    -- if ballr>0.6 then
    --   if hcm.get_ball_approach()==1 then return "ballfound" end
    -- end

    if hcm.get_ball_approach()==1 then
      return 'ballfound'
    end
    --]]
  end

end

function state.exit()
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
