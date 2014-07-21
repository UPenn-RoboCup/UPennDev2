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


local step_planner
local t_entry, t_update, t_exit
local nwaypoints, wp_id
local waypoints = {}

local target_pose
local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
local supportLeg
local ball_side = 1

local last_ph = 0

local uTorso0 = nil
local pose0 = nil

local last_step = 0



local function robocup_approach( pose, target_pose)
  --local maxStep = 0.04
  local maxStep = 0.08
  local maxTurn = 0.15
  local dist_threshold = Config.fsm.bodyRobocupFollow.th_dist
  local angle_threshold = .1

  -- Distance to the waypoint
  local rel_pose = util.pose_relative(target_pose,pose)
  local rel_dist = math.sqrt(rel_pose[1]*rel_pose[1]+rel_pose[2]*rel_pose[2])

  -- calculate walk step velocity based on ball position
  local vStep = vector.zeros(3)
  -- TODO: Adjust these constants
 
  vStep[1] = math.min(maxStep,math.max(-maxStep,rel_pose[1]*0.5))
  vStep[2] = math.min(maxStep,math.max(-maxStep,rel_pose[2]*0.5))
  vStep[3]=0

  -- Reduce speed based on how far away from the waypoint we are
  if rel_dist < 0.04 then maxStep = 0.02 end
  local scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1)
  vStep = scale * vStep

 
  return vStep, false
end


local function robocup_approach2(uLeftGlobalTarget, uRightGlobalTarget)

  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local supportLeg = mcm.get_status_supportLeg()
  local uTorsoNext

  local uLSupport = util.pose_global({Config.walk.supportX, Config.walk.supportY,0},uLeft)
  local uRSupport = util.pose_global({Config.walk.supportX, -Config.walk.supportY,0},uRight)
  local uTorsoCurrent = util.se2_interpolate(0.5, uLSupport, uRSupport)

  local pose = wcm.get_robot_pose()

  --uLeft and uRight from uTorso0
  local uLeftFromTorso = util.pose_relative(uLeft,uTorsoCurrent)
  local uRightFromTorso = util.pose_relative(uRight,uTorsoCurrent)

  local uLeftTargetFromTorso = util.pose_relative(uLeftGlobalTarget,pose)
  local uRightTargetFromTorso = util.pose_relative(uRightGlobalTarget,pose)
  
  local supportStr

  if last_step==1 then
    if supportLeg==0 then 
      uRightTargetFromTorso = util.pose_global({0, -2*Config.walk.footY,0},uLeftFromTorso)
      uLSupportNext = util.pose_global({Config.walk.supportX, Config.walk.supportY,0},uLeftFromTorso)
      uRSupportNext = util.pose_global({Config.walk.supportX, -Config.walk.supportY,0},uRightTargetFromTorso)
      uTorsoNext = util.se2_interpolate(0.5, uLSupportNext, uRSupportNext)
    else
      uLeftTargetFromTorso = util.pose_global({0, 2*Config.walk.footY,0},uRightFromTorso)
      uLSupportNext = util.pose_global({Config.walk.supportX, Config.walk.supportY,0},uLeftFromTorso)
      uRSupportNext = util.pose_global({Config.walk.supportX, -Config.walk.supportY,0},uRightTargetFromTorso)
      uTorsoNext = util.se2_interpolate(0.5, uLSupportNext, uRSupportNext)
    end
    local vStep = {uTorsoNext[1],uTorsoNext[2],0}    
    last_step=2
    return vStep,false
  elseif last_step==2 then
    return {0,0,0},true
  end

  if supportLeg==0 then 
    --Last step was left support step (right foot movement)
    --Next step should be left foot movement
    supportStr='Left foot move next'
    uLSupportNext = util.pose_global({Config.walk.supportX, Config.walk.supportY,0},uLeftFromTorso)
    uRSupportNext = util.pose_global({Config.walk.supportX, -Config.walk.supportY,0},uRightTargetFromTorso)
    uTorsoNext = util.se2_interpolate(0.5, uLSupportNext, uRSupportNext)
  else
    supportStr='Right foot move next'
    uLSupportNext = util.pose_global({Config.walk.supportX, Config.walk.supportY,0},uLeftFromTorso)
    uRSupportNext = util.pose_global({Config.walk.supportX, -Config.walk.supportY,0},uRightTargetFromTorso)
    uTorsoNext = util.se2_interpolate(0.5, uLSupportNext, uRSupportNext)
  end
  
  local vStepTarget = {uTorsoNext[1],uTorsoNext[2],0}

  local maxStep = 0.06
  vStep={0,0,0}
  vStep[1] = math.min(Config.walk.velLimitX[2],math.max(Config.walk.velLimitX[1],vStepTarget[1]))
  vStep[2] = math.min(Config.walk.velLimitY[2],math.max(Config.walk.velLimitY[1],vStepTarget[2]))

  velMag = math.sqrt(vStep[1]^2+vStep[2]^2)
  vStep[1]=vStep[1]/velMag * math.min(maxStep,velMag)
  vStep[2]=vStep[2]/velMag * math.min(maxStep,velMag)
  vStep[3]=0

  if Config.debug.approach then
    print("=====\n"..supportStr)
    print(string.format("Ball xy: %.2f %.2f",wcm.get_ball_x(),wcm.get_ball_y() ))
    print(string.format("Current: L (%.2f %.2f)  T (%.2f, %.2f)R (%.2f %.2f)",
      uLeftFromTorso[1],uLeftFromTorso[2],
      0,0,      
      uRightFromTorso[1],uRightFromTorso[2]))
    print(string.format("Target:  L (%.2f %.2f)  T (%.2f %.2f) R (%.2f %.2f)",
      uLeftTargetFromTorso[1],uLeftTargetFromTorso[2],
      uTorsoNext[1],uTorsoNext[2],
      uRightTargetFromTorso[1],uRightTargetFromTorso[2]))
  end

  if math.abs(vStep[1]-vStepTarget[1])<0.005 and
    math.abs(vStep[2]-vStepTarget[2])<0.005 then
    last_step = 1    
  end
  return vStep,false
end






local function update_velocity()
  local pose = wcm.get_robot_pose()
  local ballx = wcm.get_ball_x() 
  local bally = wcm.get_ball_y() 

  local kicktype = mcm.get_walk_kicktype()

  approachTargetX = Config.approachTargetX[kicktype+1] or 0.35
  approachTargetY = Config.approachTargetY

  local uLeftGlobalTarget, uRightGlobalTarget
  if ball_side>0 then --Align to the left foot
    uLeftGlobalTarget = util.pose_global({
        ballx - approachTargetX - Config.walk.supportX,
        bally + approachTargetY[1],
        0},pose)
    uRightGlobalTarget = util.pose_global({
        ballx - approachTargetX - Config.walk.supportX,
        bally + approachTargetY[1] -2*Config.walk.footY,
        0},pose)
  else --Align to the right foot
    uLeftGlobalTarget = util.pose_global({
      ballx - approachTargetX - Config.walk.supportX,
      bally + approachTargetY[2] +2*Config.walk.footY,
      0},pose)
    uRightGlobalTarget = util.pose_global({
      ballx - approachTargetX - Config.walk.supportX,
      bally + approachTargetY[2],
      0},pose)
  end

  local vStep,arrived = robocup_approach2(uLeftGlobalTarget, uRightGlobalTarget)
  

  local ballr = math.sqrt(ballx*ballx+bally*bally)
  local balla = math.atan2(bally,ballx)
  local walk_target_local = {ballx,bally,balla}
  local target_pose = util.pose_global(walk_target_local, pose)

  mcm.set_walk_vel(vStep)

  local t  = Body.get_time()
  local ball_elapsed = t - wcm.get_ball_t()
  

  if ballr > 1.0 then 
    print("Ball distance too away at:",ballr)
    return 'ballfar' 
  end

  if ball_elapsed <0.5 and arrived then
    print(string.format("Final ball pos: %.2f, %.2f",wcm.get_ball_x(),wcm.get_ball_y() )) 
    if ball_side==1 then
      mcm.set_walk_kickfoot(0)--left foot kick
    else
      mcm.set_walk_kickfoot(1)--right foot kick
    end
    
    return 'done'
  end
end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  local ret = nil

  t_entry = Body.get_time()
  t_update = t_entry
  
  local bally = wcm.get_ball_y()
  print(string.format("Initial ball pos:%.2f %.2f",
    wcm.get_ball_x(), wcm.get_ball_y()
    ))
  if bally<0 then
    print("Ball right")
    ball_side = -1
  else
    print("Ball left")
    ball_side = 1
  end
  last_ph = 0  
  last_step = 0
  wcm.set_robot_etastep(-1) --we're in approach


  --Determine kick types here!
--
  mcm.set_walk_kicktype(1) --strong kick default
  if Config.enable_weaker_kick then
    local ballx = wcm.get_ball_x()
    local bally = wcm.get_ball_y()
    local pose = wcm.get_robot_pose()
    local ballGlobal = util.pose_global({ballx,bally,0},pose)
    if ballGlobal[1]<0.5 then
      mcm.set_walk_kicktype(0)
    end
  end
--]]


end

function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local ret = nil
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

--not playing?
  if gcm.get_game_state()~=3 and gcm.get_game_state()~=5 and gcm.get_game_state()~=6 then return'stop' end

  local check_ph = 0.95
  local ph = mcm.get_status_ph()
  if last_ph<check_ph and ph>=check_ph then ret=update_velocity() end
  last_ph = ph
  return ret
end

function state.exit()
  print(state._NAME..' Exit' )
  wcm.set_robot_etastep(0) --out of approach
end

return state
