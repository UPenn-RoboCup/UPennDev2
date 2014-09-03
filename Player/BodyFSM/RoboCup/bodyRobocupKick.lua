local state = {}
state._NAME = ...
local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local libStep = require'libStep'
-- FSM coordination
local simple_ipc = require'simple_ipc'
local motion_ch = simple_ipc.new_publisher('MotionFSM!')


-- Get the human guided approach
require'hcm'
-- Get the robot guided approach
require'wcm'

require'mcm'
local head_ch   = simple_ipc.new_publisher('HeadFSM!')

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry  


  if Config.disable_kick or not Config.use_walkkick then
    mcm.set_walk_kickphase(0)
    mcm.set_walk_stoprequest(1)
  else

    if mcm.get_walk_kicktype()==1 then --strong kick default
      mcm.set_walk_stoprequest(1)
    end

    mcm.set_walk_kickphase(0)    
  end

end

function state.update()
  if Config.disable_kick then

    local ballx = wcm.get_ball_x() - Config.fsm.bodyRobocupApproach.target[1]
    local bally = wcm.get_ball_y()
    local ballr = math.sqrt(ballx*ballx+bally*bally)
    if ballr > 0.6 then
      return 'done'
    end
    return
  end

  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if mcm.get_walk_kickphase()==2 then
    head_ch:send'kickfollow'
    print("bodykick done!")
    return 'done'
  end 

  if mcm.get_walk_kickphase()==0 then
    if Config.use_walkkick and mcm.get_walk_kicktype()~=1 then 
      mcm.set_walk_steprequest(1)
      mcm.set_walk_kickphase(1)
    elseif mcm.get_walk_ismoving()==0 then
      mcm.set_walk_steprequest(1)
      mcm.set_walk_kickphase(1)
    end
  end
end

function state.exit()
  print(state._NAME..' Exit' )
  mcm.set_walk_kickphase(0) --now can kick again
end

return state
