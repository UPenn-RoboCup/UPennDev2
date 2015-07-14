local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
require'wcm'


local t_entry, t_update
local stage
local DEG_TO_RAD = math.pi/180


--SJ: Adult sized league doesn't need too complicated stuff
--SO let's just define params directly on the fsm files

local dqNeckLimit = {180*DEG_TO_RAD, 180*DEG_TO_RAD}




local scanVel1 = 180*DEG_TO_RAD
local scanVel2 = 60*DEG_TO_RAD
if Config.demo then
  scanVel1 = 60*DEG_TO_RAD
end


--Pitch: 25 degree down can see up to 5 meters
-- 25 is not enough from test in webots
--60 degree down can see ball right in front

function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  stage = 1
  wcm.set_ball_disable(0)
  if Config.demo then
    wcm.set_ball_backonly(0)
  else
    wcm.set_ball_backonly(1)
  end
  
  wcm.set_goal_disable(1)
  wcm.set_obstacle_enable(0)
  
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  local qNeck0 = Body.get_head_command_position()
  local headBias = hcm.get_camera_bias()
  qNeck0[1] = qNeck0[1] - headBias[1]  


  local yawTarget, pitchTarget
  if stage==1 then
    dqNeckLimit = {scanVel1, scanVel1}
    pitchTarget = 20*DEG_TO_RAD
    yawTarget = -90*DEG_TO_RAD
  elseif stage==2 then
    dqNeckLimit = {scanVel2, scanVel2}    
    pitchTarget = 20*DEG_TO_RAD
    yawTarget = -135*DEG_TO_RAD
  elseif stage==3 then
    dqNeckLimit = {scanVel2, scanVel2}    
    pitchTarget = 60*DEG_TO_RAD
    yawTarget = -135*DEG_TO_RAD
  elseif stage==4 then        
    pitchTarget = 60*DEG_TO_RAD
    yawTarget = -90*DEG_TO_RAD
  elseif stage==5 then
    dqNeckLimit = {scanVel1, scanVel1}    
    pitchTarget = 60*DEG_TO_RAD
    yawTarget = 90*DEG_TO_RAD
  elseif stage==6 then
    dqNeckLimit = {scanVel2, scanVel2}        
    pitchTarget = 60*DEG_TO_RAD
    yawTarget = 135*DEG_TO_RAD
  elseif stage==7 then
    dqNeckLimit = {scanVel2, scanVel2}        
    pitchTarget = 20*DEG_TO_RAD        
    yawTarget = 135*DEG_TO_RAD
  elseif stage==8 then
    dqNeckLimit = {scanVel2, scanVel2}        
    pitchTarget = 20*DEG_TO_RAD            
    yawTarget = 90*DEG_TO_RAD
  else
    if Config.demo then
      return 'scan'
    else

      wcm.set_ball_notvisible(1)
      print("Couldn't find the ball!!!!")
      return 'noball' --couldn't find the ball. Ball should be right behind the robot!
    end
  end

  local qNeckActual = Body.get_head_position()
  local qNeck_approach, doneNeck = 
    util.approachTol( qNeck0, {yawTarget,pitchTarget}, dqNeckLimit, dt )

  local angleErr = math.sqrt(
    (qNeckActual[1]-yawTarget-headBias[1])^2+
    (qNeckActual[2]-pitchTarget)^2
  )

  --print(angleErr*180/math.pi)
  if doneNeck and angleErr< 5*math.pi/180 then stage = stage+1 end
  -- Update the motors
--  Body.set_head_command_position(qNeck_approach)
  local headBias = hcm.get_camera_bias()  
  Body.set_head_command_position({qNeck_approach[1]+headBias[1],qNeck_approach[2]})
  wcm.set_ball_tlook(t)

	-- Check if we found the ball
  local ball_elapsed = t - wcm.get_ball_t()
  if ball_elapsed < 0.1 then
    print("Ball found")

    local ballx = wcm.get_ball_x()
    local bally = wcm.get_ball_y()  
     print("ball xy:",ballx,bally)

    return 'ballfound' --if ball found exit
  end
  
end

function state.exit()
  print("HeadBackScan time: ",Body.get_time()-t_entry)
  print(state._NAME..' Exit')
  wcm.set_ball_backonly(0)
end

return state

