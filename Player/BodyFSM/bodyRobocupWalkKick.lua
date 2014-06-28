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


local kick_started

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  if not Config.disable_kick then
    mcm.set_walk_steprequest(1)
  end
  kick_started = false
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

  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if mcm.get_walk_ismoving()==0 then
    if kick_started then 
      if mcm.get_walk_kicktype()==1 then
        return 'testdone' --this means testing mode (don't run body fsm)      
      else
        return 'done'
      end
    end
  else
    kick_started = true
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
