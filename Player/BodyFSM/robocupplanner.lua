local robocupplanner={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'


function robocupplanner.getGoalieTargetPose(pose,ballGlobal)
  --Defending goal is ALWAYS at (-4.5,0,0)


  local goalL = {-Config.world.xBoundary,-Config.world.goalWidth/2}
  local goalR = {-Config.world.xBoundary,Config.world.goalWidth/2}


  local ballFromGoalL = {ballGlobal[1] -goalL[1], ballGlobal[2]-goalL[2]}
  local ballFromGoalR = {ballGlobal[1] -goalR[1], ballGlobal[2]-goalR[2]}
  
  local ballGoalAngleL = math.atan2(-ballFromGoalL[2],-ballFromGoalL[1])
  local ballGoalAngleR = math.atan2(-ballFromGoalR[2],-ballFromGoalR[1])

  local ballGoalAngleCenter = (ballGoalAngleL+ballGoalAngleR)/2

  -- x = ball(1) + t*cos(angle) = -Config.world.xBoundary
  -- y = ball(2) + t*sin(angle) 

  
  local factor2 = Config.world.goalieFactor or 0.88 --Goalie pos
  local factor1 = factor2 - 0.02 --max ball pos


  ballGlobal[1] = math.max(  -Config.world.xBoundary * factor1, ballGlobal[1])

  local t = (-Config.world.xBoundary*factor2 - ballGlobal[1])/math.cos(ballGoalAngleCenter) 

  local target_position_x = -Config.world.xBoundary * factor2
  local target_position_y = ballGlobal[2] + math.sin(ballGoalAngleCenter)*t
  local target_position_a = pose[3]


  target_position_y = math.max(
    -Config.world.goalWidth/2 * 0.8, math.min(
      Config.world.goalWidth/2 * 0.8 , target_position_y))

  --
  --Facing towards the ball
  local ballfromtarget = {ballGlobal[1]-target_position_x,ballGlobal[2]-target_position_y}
  target_position_a = math.atan2(ballfromtarget[2],ballfromtarget[1])
 --]] 

  return {target_position_x, target_position_y, target_position_a}
end



function robocupplanner.getTargetPose(pose,ballGlobal)
  if Config.demo  then return robocupplanner.getDemoTargetPose(pose,ballGlobal) end
  if Config.backward_approach then return robocupplanner.getTargetPoseBackward(pose,ballGlobal) end

  local goalL = {Config.world.xBoundary,Config.world.goalWidth/2}
  local goalR = {Config.world.xBoundary,-Config.world.goalWidth/2}
  local goal = {Config.world.xBoundary,0}

  local goaldir = vector.new({goal[1]-ballGlobal[1],goal[2]-ballGlobal[2],0})
  local goaldist = math.sqrt(goaldir[1]*goaldir[1]+goaldir[2]*goaldir[2])
  local angleGoalBall = math.atan2(goaldir[2],goaldir[1])

  local goaldirL = vector.new({goalL[1]-ballGlobal[1],goalL[2]-ballGlobal[2],0})
  local goaldistL = math.sqrt(goaldirL[1]*goaldirL[1]+goaldirL[2]*goaldirL[2])
  local angleGoalBallL = math.atan2(goaldirL[2],goaldirL[1])

  local goaldirR = vector.new({goalR[1]-ballGlobal[1],goalR[2]-ballGlobal[2],0})
  local goaldistR = math.sqrt(goaldirR[1]*goaldirR[1]+goaldirR[2]*goaldirR[2])
  local angleGoalBallR = math.atan2(goaldirR[2],goaldirR[1])

  local angleRobotBall = math.atan2(pose[2]-ballGlobal[2],pose[1]-ballGlobal[1])
  local distRobotBall = math.sqrt( (pose[1]-ballGlobal[1])^2+(pose[2]-ballGlobal[2])^2 )
  
  local circleR = 1
  local angle_tangent = math.acos(circleR / math.max(circleR,distRobotBall))
  local angleCircle = util.mod_angle(angleRobotBall-angleGoalBall)


  if math.abs(angleCircle)>90*math.pi/180 and math.abs(util.mod_angle(pose[3]-angleGoalBallR))<90*math.pi/180 then
    --Behind the ball and approximately facing the goalpost
    --Directly approach to the kick position
    local kickoffset = 0.5
    kickpos = ballGlobal - kickoffset * goaldir/goaldist
    kickpos[3] = math.atan2(goaldir[2],goaldir[1])

  elseif math.abs(angleCircle)>150*math.pi/180 then --Robot is behind the ball!
    --Just approach
    local kickoffset = 0.5
    kickpos = ballGlobal - kickoffset * goaldir/goaldist
    kickpos[3] = math.atan2(goaldir[2],goaldir[1])
  elseif angleCircle<0 then --Robot is on the right side
    local angleTangent = angleRobotBall - angle_tangent
    local angleTarget = angleTangent
    local angleMargin = 20*math.pi/180

    --Make the robot circle around, if it already reached the tangent point    
    if angleRobotBall<angleTangent + angleMargin then
      angleTarget = angleRobotBall - angleMargin
    end

    kickpos = ballGlobal
    kickpos[1] = kickpos[1] + math.cos(angleTarget)*circleR
    kickpos[2] = kickpos[2] + math.sin(angleTarget)*circleR
    kickpos[3] = util.mod_angle(angleTangent - math.pi/2)
  else --Robot is on the left side 
    local angleTangent = angleRobotBall + angle_tangent
    local angleTarget = angleTangent
    local angleMargin = 20*math.pi/180
    
    --Make the robot circle around, if it already reached the tangent point    
    if angleRobotBall>angleTangent - angleMargin then
      angleTarget = angleRobotBall + angleMargin
    end

    kickpos = ballGlobal
    kickpos[1] = kickpos[1] + math.cos(angleTarget)*circleR
    kickpos[2] = kickpos[2] + math.sin(angleTarget)*circleR
    kickpos[3] = util.mod_angle(angleTarget + math.pi/2)
  end

  return kickpos
end

function robocupplanner.getTargetPoseBackward(pose,ballGlobal)
  --This target pose lets robot walk backward towards the ball

  local goalL = {4.5,Config.world.goalWidth/2}
  local goalR = {4.5,-Config.world.goalWidth/2}
  local goal = {4.5,0}

  local goaldir = vector.new({goal[1]-ballGlobal[1],goal[2]-ballGlobal[2],0})
  local goaldist = math.sqrt(goaldir[1]*goaldir[1]+goaldir[2]*goaldir[2])
  local angleGoalBall = math.atan2(goaldir[2],goaldir[1])

  local goaldirL = vector.new({goalL[1]-ballGlobal[1],goalL[2]-ballGlobal[2],0})
  local goaldistL = math.sqrt(goaldirL[1]*goaldirL[1]+goaldirL[2]*goaldirL[2])
  local angleGoalBallL = math.atan2(goaldirL[2],goaldirL[1])

  local goaldirR = vector.new({goalR[1]-ballGlobal[1],goalR[2]-ballGlobal[2],0})
  local goaldistR = math.sqrt(goaldirR[1]*goaldirR[1]+goaldirR[2]*goaldirR[2])
  local angleGoalBallR = math.atan2(goaldirR[2],goaldirR[1])



  local angleRobotBall = math.atan2(pose[2]-ballGlobal[2],pose[1]-ballGlobal[1])
  local distRobotBall = math.sqrt( (pose[1]-ballGlobal[1])^2+(pose[2]-ballGlobal[2])^2 )
  
  local circleR = Config.fsm.bodyRobocupFollow.circleR or 1.5
  local kickoffset = Config.fsm.bodyRobocupFollow.kickoffset or 0.5


  local angle_tangent = math.acos(circleR / math.max(circleR,distRobotBall))
  local angleCircle = util.mod_angle(angleRobotBall-angleGoalBall)

  if math.abs(angleCircle)>150*math.pi/180 then --Robot is behind the ball!
    --Just approach
    
    kickpos = ballGlobal - kickoffset * goaldir/goaldist
    kickpos[3] = angleGoalBall

  elseif angleCircle<0 then --Robot is on the right side
    local angleTangent = angleRobotBall - angle_tangent
    local angleTarget = angleTangent
    local angleMargin = 20*math.pi/180

    --Make the robot circle around, if it already reached the tangent point    
    if angleRobotBall<angleTangent + angleMargin then
      angleTarget = angleRobotBall - angleMargin
    end

    kickpos = ballGlobal
    kickpos[1] = kickpos[1] + math.cos(angleTarget)*circleR
    kickpos[2] = kickpos[2] + math.sin(angleTarget)*circleR
    kickpos[3] = angleGoalBall
  else --Robot is on the left side 
    local angleTangent = angleRobotBall + angle_tangent
    local angleTarget = angleTangent
    local angleMargin = 20*math.pi/180
    
    --Make the robot circle around, if it already reached the tangent point    
    if angleRobotBall>angleTangent - angleMargin then
      angleTarget = angleRobotBall + angleMargin
    end

    kickpos = ballGlobal
    kickpos[1] = kickpos[1] + math.cos(angleTarget)*circleR
    kickpos[2] = kickpos[2] + math.sin(angleTarget)*circleR
    kickpos[3] = angleGoalBall
  end

  return kickpos
end

function robocupplanner.getDemoTargetPose(pose,ballGlobal)
  --Direct pose (not turning to the post)
  local angleRobotBall = math.atan2(pose[2]-ballGlobal[2],pose[1]-ballGlobal[1])
  local distRobotBall = math.sqrt( (pose[1]-ballGlobal[1])^2+(pose[2]-ballGlobal[2])^2 )
  local kickoffset = 0.50

  kickpos = ballGlobal
  kickpos[1] = kickpos[1] + math.cos(angleRobotBall)*kickoffset
  kickpos[2] = kickpos[2] + math.sin(angleRobotBall)*kickoffset
  kickpos[3] = util.mod_angle(angleRobotBall+math.pi)

  return kickpos

end


function robocupplanner.getVelocity(pose,target_pose )  
  -- Distance to the waypoint
  local homeRelative = util.pose_relative(target_pose,pose)
  local rHomeRelative = math.sqrt(homeRelative[1]^2+homeRelative[2]^2)
  local aHomeRelative = util.mod_angle(math.atan2(homeRelative[2],homeRelative[1]))
  local homeRot = math.abs(aHomeRelative)
  local homeRotBack = math.abs(util.mod_angle(aHomeRelative - math.pi))

  if math.abs(homeRelative[1])<0.4 and math.abs(homeRelative[2])<0.4 and 
    math.abs(util.mod_angle(pose[3]-target_pose[3]))<0.1 then
  	return {0,0,0},true
  end

  rTurn = 0.6
  if Config.backward_approach and homeRot>homeRotBack then --walking backward means less turning
    aHomeRelative = util.mod_angle(aHomeRelative - math.pi) --turn to face back to the target
    rTurn = 1.5 
  end
  
  aTurn = math.exp(-0.5*(rHomeRelative/rTurn)^2)
  local vx,vy,va = 0,0,0

  va = 0.5* (aTurn*util.mod_angle(homeRelative[3])) + (1-aTurn)*aHomeRelative
  if rHomeRelative>1.0 then
    if math.abs(va)<0.2 then maxStep,maxA = 0.20,0.05
    else maxStep,maxA = 0, 0.2 end
  elseif rHomeRelative > 0.5 then  maxStep,maxA = 0.10, 0.1
  else  maxStep,maxA = 0.05,0.2 end
  vx = maxStep * homeRelative[1]/rHomeRelative
  vy = maxStep * homeRelative[2]/rHomeRelative
  va = math.min(maxA, math.max(-maxA, va))
  return{vx,vy,va},false
end

function robocupplanner.getVelocityGoalie(pose,target_pose , threshold)  
  -- Distance to the waypoint

  threshold = threshold or 0.2
  local homeRelative = util.pose_relative(target_pose,pose)
  local rHomeRelative = math.sqrt(homeRelative[1]^2+homeRelative[2]^2)
  local aHomeRelative = util.mod_angle(math.atan2(homeRelative[2],homeRelative[1]))
  local homeRot = math.abs(aHomeRelative)
  local homeRotBack = math.abs(util.mod_angle(aHomeRelative - math.pi))

  if math.abs(homeRelative[1])<threshold and 
    math.abs(homeRelative[2])<threshold and
    math.abs(util.mod_angle(homeRelative[2])) < 20*math.pi/180     then
    return {0,0,0},true
  end

  local vx,vy,va = 0,0,0

  va = 0.3*util.mod_angle(homeRelative[3])
  maxStep,maxA = 0.05,0.2
  vx = maxStep * homeRelative[1]/rHomeRelative
  vy = maxStep * homeRelative[2]/rHomeRelative
  va = math.min(maxA, math.max(-maxA, va))
  return{vx,vy,va},false
end



return robocupplanner