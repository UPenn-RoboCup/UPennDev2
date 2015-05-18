local state = {}
state._NAME = ...


require'wcm'
require'hcm'
local Body = require'Body'
local util = require'util'
local simple_ipc = require'simple_ipc'
local arm_ch = simple_ipc.new_publisher('ArmFSM!')

local t_entry, t_update
local stage
local DEG_TO_RAD = math.pi/180

local dqNeckLimit = {180*DEG_TO_RAD, 180*DEG_TO_RAD}

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  stage = 1
--  wcm.set_ball_disable(0)
end

function state.update()
  --print(state._NAME..' Update' )
  local t = Body.get_time()
  local dt = t - t_update
  t_update = t

  local qNeck0 = Body.get_head_command_position()
  local headBias = hcm.get_camera_bias()
  qNeck0[1] = qNeck0[1] - headBias[1]  

  local yawTarget, pitchTarget

  if stage==1 then
    pitchTarget = 0*DEG_TO_RAD
    yawTarget = 0*DEG_TO_RAD
  elseif stage==2 then
    pitchTarget = 26.9*DEG_TO_RAD
    yawTarget = 0*DEG_TO_RAD
  elseif stage==3 then
    pitchTarget = 0*DEG_TO_RAD
    yawTarget = 0*DEG_TO_RAD
  else
    return 'lookaround'
  end

  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck0, {yawTarget,pitchTarget}, dqNeckLimit, 0.0032 )
  if doneNeck then stage = stage+1 end

  local headBias = hcm.get_camera_bias()
  Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})
  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state

