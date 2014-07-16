local t_entry, t_update
local state = {}
state._NAME = ...

local Body = require'Body'
local vector = require'vector'
local HT = require'libHeadTransform'
local util = require'util'
require'wcm'
require'gcm'

local ball_radius = Config.world.ballDiameter / 2
local tLost = Config.fsm.headTrack.tLost
local timeout = Config.fsm.headTrack.timeout
local dqNeckLimit = Config.fsm.dqNeckLimit or {90*DEG_TO_RAD, 90*DEG_TO_RAD}


local pitchMin = Config.head.pitchMin
local pitchMax = Config.head.pitchMax
local yawMin = Config.head.yawMin
local yawMax = Config.head.yawMax

function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()

  if gcm.get_game_state()==4 then return'teleop' end
  
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()

  local dt = t - t_update
  -- Save this at the last update time
  t_update = t



  --local ball_elapsed = t - wcm.get_ball_t()
  --How long time we have TRIED to look at the ball but couldn't detect the ball?
  local ball_elapsed = wcm.get_ball_tlook() - wcm.get_ball_t()
  
  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  local headBias = hcm.get_camera_bias()
  qNeck[1] = qNeck[1] - headBias[1]  

  yaw,pitch = 0,0

  -- Go!
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck, {yaw,pitch}, dqNeckLimit, dt )
  
  local headBias = hcm.get_camera_bias()
  Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})
  if doneNeck then return "done" end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
