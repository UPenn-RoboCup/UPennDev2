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
local dqNeckLimit = Config.fsm.dqNeckLimit

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

  if ball_elapsed > tLost then --ball lost
    print "Ball lost"
    return 'balllost'
  end

  local ballX, ballY = wcm.get_ball_x() - 0.5 , wcm.get_ball_y()
  local yaw, pitch = HT.ikineCam(ballX, ballY, ball_radius)

  -- print('Ball dist:', math.sqrt(ballX*ballX + ballY*ballY))
  -- Look at Goal
  if not Config.demo and not Config.use_gps_pose and t-t_entry > timeout then
    -- If robot is close to the ball then do not look up
    if math.sqrt(ballX*ballX + ballY*ballY) > Config.fsm.headTrack.dist_th then
      if gcm.get_game_role()==0 then
        --Goalie don't look goals
      else
    	  return 'timeout'
      end
    end
  end

  -- Clamp
  yaw = math.min(math.max(yaw, yawMin), yawMax)
  pitch = math.min(math.max(pitch, pitchMin), pitchMax)

  --If ball is in front just look down
  if math.sqrt(ballX*ballX + ballY*ballY) <0.5  then yaw = 0 end

  -- Grab where we are
  local qNeck = Body.get_head_command_position()
  local headBias = hcm.get_camera_bias()
  qNeck[1] = qNeck[1] - headBias[1]  

  -- Go!
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck, {yaw,pitch}, dqNeckLimit, dt )
    
  -- Update the motors
--  Body.set_head_command_position(qNeck_approach)

  local headBias = hcm.get_camera_bias()
  Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})
  wcm.set_ball_tlook(t)
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
