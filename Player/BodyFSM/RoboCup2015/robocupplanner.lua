local robocupplanner={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'wcm'

function evaluate_goal_kickangle(ballGlobal)
  local obstacle_dist_threshold = 0.1
--  local kick_deviation_angle = 10*math.pi/180
  local kick_deviation_angle = 5*math.pi/180

  local goalL = {Config.world.xBoundary,Config.world.goalWidth/2}
  local goalR = {Config.world.xBoundary,-Config.world.goalWidth/2}

--Make goal smaller (to compensate for the localization error)
  local goalL = {Config.world.xBoundary,Config.world.goalWidth/2 * 0.9}
  local goalR = {Config.world.xBoundary,-Config.world.goalWidth/2 * 0.9}
  local goalC = {Config.world.xBoundary,0}


  wcm.set_robot_goal1({goalL[1],goalL[2]})
  wcm.set_robot_goal2({goalR[1],goalR[2]})
  wcm.set_robot_goal3({goalC[1],goalC[2]})

  local goalAngleL = math.atan2(goalL[2]-ballGlobal[2],goalL[1]-ballGlobal[1])-kick_deviation_angle
  local goalAngleR = math.atan2(goalR[2]-ballGlobal[2],goalR[1]-ballGlobal[1])+kick_deviation_angle
  local goalAngleC = math.atan2(goalC[2]-ballGlobal[2],goalC[1]-ballGlobal[1])

  if goalAngleL-goalAngleR<0 then return 0 end

  obstacle_num = wcm.get_obstacle_num()

  --max_obstacle_dist = 0
  --max_obstacle_dist_angle = nil

  max_obstacle_angle = 0
  max_obstacle_angle_kickangle = nil

  for a=0,10 do 
    local angle = goalAngleR +  (a/10) * (goalAngleL-goalAngleR)
    local obs_angle_min = math.pi
    for i=1,obstacle_num do   
      local v =wcm['get_obstacle_v'..i]()
      local rel_obs_x = v[1]-ballGlobal[1]
      local rel_obs_y = v[2]-ballGlobal[2]
      local obs_dist = math.sqrt(rel_obs_x*rel_obs_x + rel_obs_y*rel_obs_y)
      local obs_angle = math.atan2(rel_obs_y, rel_obs_x)
      local angle_diff = math.max(0, math.abs(util.mod_angle(obs_angle-angle)) - kick_deviation_angle)
      --local obs_closest_dist = math.sin(angle_diff)*obs_dist
      --if angle_diff>math.pi/2 then obs_closest_dist = 999 end
      --obs_dist_min = math.min(obs_dist_min, obs_closest_dist)  
      obs_angle_min = math.min(angle_diff, obs_angle_min)
    end

    if Config.assume_goalie_blocking then
        local v =wcm.get_robot_goal1()
        local rel_obs_x = v[1]-ballGlobal[1]
        local rel_obs_y = v[2]-ballGlobal[2]
        local obs_dist = math.sqrt(rel_obs_x*rel_obs_x + rel_obs_y*rel_obs_y)
        local obs_angle = math.atan2(rel_obs_y, rel_obs_x)
        local angle_diff = math.max(0, math.abs(util.mod_angle(obs_angle-angle)) - kick_deviation_angle)

        --local obs_closest_dist = math.sin(angle_diff)*obs_dist
        --if angle_diff>math.pi/2 then obs_closest_dist = 999 end
        --obs_dist_min = math.min(obs_dist_min, obs_closest_dist)  
        obs_angle_min = math.min(angle_diff, obs_angle_min)

        local goal_angle1 = obs_angle

        local v =wcm.get_robot_goal2()
        local rel_obs_x = v[1]-ballGlobal[1]
        local rel_obs_y = v[2]-ballGlobal[2]
        local obs_dist = math.sqrt(rel_obs_x*rel_obs_x + rel_obs_y*rel_obs_y)
        local obs_angle = math.atan2(rel_obs_y, rel_obs_x)

        local angle_diff = math.max(0, math.abs(util.mod_angle(obs_angle-angle)) - kick_deviation_angle)
        --local obs_closest_dist = math.sin(angle_diff)*obs_dist
        --if angle_diff>math.pi/2 then obs_closest_dist = 999 end
        --obs_dist_min = math.min(obs_dist_min, obs_closest_dist)  
        obs_angle_min = math.min(angle_diff, obs_angle_min)
        local goal_angle2 = obs_angle

        local goal_anglec = (goal_angle1+goal_angle2)/2

        local shift_factor = Config.enemy_goalie_shift_factor or 0.2
        if goal_anglec>0 then --we're on the right side
          goal_anglec = (1-shift_factor)*goal_anglec +shift_factor*goal_angle2
        else
          goal_anglec = (1-shift_factor)*goal_anglec +shift_factor*goal_angle1
        end

--      print("angles:",goal_angle1*180/math.pi,goal_anglec*180/math.pi,goal_angle2*180/math.pi)

        local angle_diff = math.max(0, 
          math.abs(util.mod_angle(goal_anglec-angle)) - kick_deviation_angle)
        obs_angle_min = math.min(angle_diff, obs_angle_min)
    else
      --Goalie and two goalposts
      for i=1,3 do   
        local v =wcm['get_robot_goal'..i]()
        local rel_obs_x = v[1]-ballGlobal[1]
        local rel_obs_y = v[2]-ballGlobal[2]
        local obs_dist = math.sqrt(rel_obs_x*rel_obs_x + rel_obs_y*rel_obs_y)
        local obs_angle = math.atan2(rel_obs_y, rel_obs_x)
        local angle_diff = math.max(0, math.abs(util.mod_angle(obs_angle-angle)) - kick_deviation_angle)
        --local obs_closest_dist = math.sin(angle_diff)*obs_dist
        --if angle_diff>math.pi/2 then obs_closest_dist = 999 end
        --obs_dist_min = math.min(obs_dist_min, obs_closest_dist)  
        obs_angle_min = math.min(angle_diff, obs_angle_min)
      end
    end

    --[[
    if max_obstacle_dist<obs_dist_min then
      max_obstacle_dist=obs_dist_min
      max_obstacle_dist_angle = angle
    end
    --]]
    if max_obstacle_angle < obs_angle_min then
      max_obstacle_angle = obs_angle_min
      max_obstacle_angle_kickangle = angle
    end    
  end

--  return (goalAngleL-goalAngleR) * (obs_clear_angle_count) / 11, max_obstacle_dist_angle
  return max_obstacle_angle, max_obstacle_angle_kickangle
end

function evaluate_kickangle(ballGlobal,angle, kick_deviation_angle)
  local ballX_threshold1 = Config.ballX_threshold1 or -2.5
  local ballX_threshold2 = Config.ballX_threshold2 or 0.5

  local kick_distance = 4.0
  local kick_distance_max = 5.0



  if ballGlobal[1]<ballX_threshold1 then
    if Config.debug.planning then print("Ball pos 0",ballGlobal[1]) end
    kick_distance = 4.0
    kick_distance_max = 5.0
  elseif ballGlobal[1]<ballX_threshold2 then
    if Config.debug.planning then  print("Ball pos 1",ballGlobal[1]) end
    kick_distance = 2.0
    kick_distance_max = 3.0
  else
    if Config.debug.planning then print("Ball pos for final kick",ballGlobal[1]) end
    kick_distance = 5.0
    kick_distance_max = 5.0
  end

--  local kick_deviation_angle = 10*math.pi/180
 
--  if not kick_deviation_angle then kick_deviation_angle = 5*math.pi/180 end
  if not kick_deviation_angle then kick_deviation_angle = 10*math.pi/180 end
 
  --ball radius: 0.11, obstacle radius 0.10

  local obstacle_dist_threshold = 0.22
  local min_obs_dist = math.huge

  local ballEndPos={ballGlobal[1]+kick_distance*math.cos(angle),ballGlobal[2]+kick_distance*math.sin(angle)}
  local ballEndPosMax={ballGlobal[1]+kick_distance_max*math.cos(angle),ballGlobal[2]+kick_distance_max*math.sin(angle)}

  obstacle_num = wcm.get_obstacle_num()  

  for i=1,obstacle_num do 
    local v =wcm['get_obstacle_v'..i]()
    local rel_obs_x = v[1]-ballGlobal[1]
    local rel_obs_y = v[2]-ballGlobal[2]
    local obs_dist = math.sqrt(rel_obs_x*rel_obs_x + rel_obs_y*rel_obs_y)
    local obs_angle = math.atan2(rel_obs_y, rel_obs_x)

    local obs_tangent_dist = obs_dist * math.cos(math.abs(obs_angle-angle))
    local angle_diff = math.max(0, math.abs(util.mod_angle(obs_angle-angle)) - kick_deviation_angle)
    local obs_closest_dist 

    if kick_distance_max<obs_tangent_dist then
      local ballClosestPosRel = {
        obs_dist - kick_distance_max*math.cos(angle_diff),  
        kick_distance_max*math.sin(angle_diff)  
      }
      obs_closest_dist = math.sqrt(ballClosestPosRel[1]^2+ballClosestPosRel[2]^2)
    else
      obs_closest_dist = math.sin(angle_diff)*obs_dist
      if angle_diff>math.pi/2 then obs_closest_dist = math.huge end
    end
    min_obs_dist = math.min(min_obs_dist, obs_closest_dist)
  end

  local maxY_R = ballGlobal[2] + kick_distance_max * math.sin(angle-kick_deviation_angle)
  local maxY_L = ballGlobal[2] + kick_distance_max * math.sin(angle+kick_deviation_angle)
  local Ydist_L = Config.world.yBoundary - maxY_R
  local Ydist_R = maxY_L + Config.world.yBoundary
  local min_borderY_dist = math.max(0, math.min(Ydist_L,Ydist_R))

  local outstr=string.format("Kick angle %d :obs %.2f border %.2f",angle*180/math.pi, min_obs_dist, min_borderY_dist)
  
--  if min_obs_dist>0.5 and min_borderY_dist>0.5 then   

  if min_obs_dist>0.5 and min_borderY_dist>0.9 then       

    daGoal, kickAngle2 = evaluate_goal_kickangle(ballEndPos)
    if Config.debug.planning then 
      print(string.format("%s goalAngle %d",outstr,daGoal*180/math.pi))            
--      print(string.format("%s max obstacle dist: %d",outstr,daGoal*180/math.pi))            
    end
    if daGoal>0 then return daGoal, kickAngle2 , ballEndPos end
  else
    if Config.debug.planning then  print(outstr) end
    return
  end    
end


function robocupplanner.getKickAngle(pose,ballGlobal,prevKickAngle)
  local max_score = -math.huge
  local max_score_angle, max_score_angle2, best_ballEndPos1

  local ballX_threshold2 = Config.ballX_threshold2 or 0.5

  if ballGlobal[1]>ballX_threshold2 then
    if Config.debug.planning then print("Ball close, one stage planning") end
    --Ball close, do a single-stage planning
    daGoal, kickAngle = evaluate_goal_kickangle(ballGlobal)

    if not kickAngle then 
      kickAngle = prevKickAngle or 0
      kickAngle2 = prevKickAngle or 0
    end

    wcm.set_robot_kickneeded(1)
    wcm.set_robot_kickangle1(kickAngle)
    wcm.set_robot_kickangle2(kickAngle)

    wcm.set_robot_ballglobal2({
          ballGlobal[1] + 3.0 *math.cos(kickAngle),
          ballGlobal[2] + 3.0 *math.sin(kickAngle)
          })

    wcm.set_robot_ballglobal3({
          ballGlobal[1] + 3.0 *math.cos(kickAngle),
          ballGlobal[2] + 3.0 *math.sin(kickAngle)
          })

    return kickAngle
  else
    --Two-stage planning

    local score,kickangle2 
    for a = -8,8 do
      local angle = a*10*math.pi/180
      local score, kickangle2,ballEndPos = evaluate_kickangle(ballGlobal,angle)
      if score then
        if score>max_score then
          max_score_angle = angle
          max_score_angle2 = kickangle2
          best_ballEndPos1 = ballEndPos
          max_score = score
        end
      end
    end

    if max_score_angle then
      if Config.debug.planning then
        print(string.format("Best kick angle: first %d and then %d",
          max_score_angle*180/math.pi, max_score_angle2*180/math.pi))
      end

      local use_new_angle = false
      if not prevKickAngle then use_new_angle = true 
      else
        local anglediff1 = math.abs(util.mod_angle(max_score_angle-prevKickAngle))
        if anglediff1<20*math.pi/180 then use_new_angle = true end
      end
      if use_new_angle then
        wcm.set_robot_kickneeded(2)
        wcm.set_robot_kickangle1(max_score_angle)
        wcm.set_robot_kickangle2(max_score_angle2)
        wcm.set_robot_ballglobal2({best_ballEndPos1[1],best_ballEndPos1[2]})

        wcm.set_robot_ballglobal3({
          best_ballEndPos1[1] + 3.0 *math.cos(max_score_angle2),
          best_ballEndPos1[2] + 3.0 *math.sin(max_score_angle2)
          })
        return max_score_angle
      else
        return prevKickAngle
      end
    else
      print("No possible route")      
      return
    end

   end
end








function robocupplanner.getTargetPose(pose,ballGlobal,kickangle)


  local angleRobotBall = math.atan2(pose[2]-ballGlobal[2],pose[1]-ballGlobal[1])
  local distRobotBall = math.sqrt( (pose[1]-ballGlobal[1])^2+(pose[2]-ballGlobal[2])^2 )
  
  local circleR = Config.fsm.bodyRobocupFollow.circleR or 1
  local kickoffset = Config.fsm.bodyRobocupFollow.kickoffset or 0.75
  local kickoffsetMin = Config.fsm.bodyRobocupFollow.kickoffsetMin or 0.4

  local angle_tangent = math.acos(circleR / math.max(circleR,distRobotBall))
  local angleCircle = util.mod_angle(angleRobotBall-kickangle)
  local rotate = 0

  if Config.demo  then 
    kickpos = ballGlobal + kickoffset * vector.new({math.cos(angleRobotBall),math.sin(angleRobotBall),0})
    kickpos[3] = util.mod_angle(angleRobotBall+math.pi)
    return kickpos,0 
  end
  

  if Config.backward_approach then return robocupplanner.getTargetPoseBackward(pose,ballGlobal) end

  if math.abs(angleCircle)>90*math.pi/180 and math.abs(util.mod_angle(pose[3]-kickangle))<90*math.pi/180 then
    --Behind the ball and approximately facing the goalpost
    --Directly approach to the kick position
  
    local kickoffset2 = math.max(math.min(distRobotBall,kickoffset),kickoffsetMin)   
    kickpos = ballGlobal - kickoffset2 * vector.new({math.cos(kickangle),math.sin(kickangle),0})
    kickpos[3] = kickangle
    rotate = 0

  elseif math.abs(angleCircle)>150*math.pi/180 then --Robot is behind the ball!
    --Just approach
    local kickoffset2 = math.max(math.min(distRobotBall,kickoffset),kickoffsetMin)   
    kickpos = ballGlobal - kickoffset2 * vector.new({math.cos(kickangle),math.sin(kickangle),0})
    kickpos[3] = kickangle
    rotate = 0

  elseif angleCircle<0 then --Robot is on the right side
    local angleTangent = angleRobotBall - angle_tangent
    local angleTarget = angleTangent
    local angleMargin = 45*math.pi/180

    --Make the robot circle around, if it already reached the tangent point    
    if angleRobotBall<angleTangent + angleMargin then
      angleTarget = angleRobotBall - angleMargin
    end

    kickpos = ballGlobal
    kickpos[1] = kickpos[1] + math.cos(angleTarget)*circleR
    kickpos[2] = kickpos[2] + math.sin(angleTarget)*circleR
    kickpos[3] = util.mod_angle(angleTangent - math.pi/2)
    rotate = 1

  else --Robot is on the left side 
    local angleTangent = angleRobotBall + angle_tangent
    local angleTarget = angleTangent
    local angleMargin = 45*math.pi/180
    
    --Make the robot circle around, if it already reached the tangent point    
    if angleRobotBall>angleTangent - angleMargin then
      angleTarget = angleRobotBall + angleMargin
    end

    kickpos = ballGlobal
    kickpos[1] = kickpos[1] + math.cos(angleTarget)*circleR
    kickpos[2] = kickpos[2] + math.sin(angleTarget)*circleR
    kickpos[3] = util.mod_angle(angleTarget + math.pi/2)
    rotate = -1

  end

  return kickpos,rotate
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
  
  local circleR = Config.fsm.bodyRobocupFollow.circleR or 1
  local kickoffset = Config.fsm.bodyRobocupFollow.kickoffset or 0.5

  local angle_tangent = math.acos(circleR / math.max(circleR,distRobotBall))
  local angleCircle = util.mod_angle(angleRobotBall-angleGoalBall)
  local rotate = 0
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
    rotate = 1
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
    rotate = -1
  end

  return kickpos,rotate
end


function robocupplanner.getVelocity(pose,target_pose,rotate)  
  -- Distance to the waypoint
  local homeRelative = util.pose_relative(target_pose,pose)
  local rHomeRelative = math.sqrt(homeRelative[1]^2+homeRelative[2]^2)
  local aHomeRelative = util.mod_angle(math.atan2(homeRelative[2],homeRelative[1]))
  local homeRot = math.abs(aHomeRelative)
  local homeRotBack = math.abs(util.mod_angle(aHomeRelative - math.pi))

  if rotate==0 then
    if math.abs(homeRelative[1])<0.4 and math.abs(homeRelative[2])<0.4 and 
      math.abs(util.mod_angle(pose[3]-target_pose[3]))<0.1 then
      return {0,0,0},true
    end
  end

  rTurn = 0.6
  if Config.backward_approach and homeRot>homeRotBack then --walking backward means less turning
    aHomeRelative = util.mod_angle(aHomeRelative - math.pi) --turn to face back to the target
    rTurn = 1.5 
  end
  
  aTurn = math.exp(-0.5*(rHomeRelative/rTurn)^2)
  local vx,vy,va = 0,0,0

  va = 0.5* (aTurn*util.mod_angle(homeRelative[3])) + (1-aTurn)*aHomeRelative

  if rHomeRelative>0.6 then
    if math.abs(va)<0.2 then maxStep,maxA = 0.20,0.05
    else maxStep,maxA = 0, 0.2 end
  elseif rHomeRelative > 0.3 then  maxStep,maxA = 0.10, 0.1
  else  maxStep,maxA = 0.05,0.2 end


  vx = maxStep * homeRelative[1]/rHomeRelative
  vy = maxStep * homeRelative[2]/rHomeRelative
  va = math.min(maxA, math.max(-maxA, va))
  return{vx,vy,va},false
end

function robocupplanner.getGoalieTargetPose(pose,ballGlobal)
  --Defending goal is ALWAYS at (-4.5,0,0)
  local goalL = {-Config.world.xBoundary,-Config.world.goalWidth/2}
  local goalR = {-Config.world.xBoundary,Config.world.goalWidth/2}
  local ballFromGoalL = {ballGlobal[1] -goalL[1], ballGlobal[2]-goalL[2]}
  local ballFromGoalR = {ballGlobal[1] -goalR[1], ballGlobal[2]-goalR[2]}
  local ballGoalAngleL = math.atan2(-ballFromGoalL[2],-ballFromGoalL[1])
  local ballGoalAngleR = math.atan2(-ballFromGoalR[2],-ballFromGoalR[1])
  local ballGoalAngleCenter = (ballGoalAngleL+ballGoalAngleR)/2


  
  local ballYFactor = Config.ballYFactor or 1.0
  

  -- x = ball(1) + t*cos(angle) = -Config.world.xBoundary
  -- y = ball(2) + t*sin(angle) 
  local factor2 = Config.world.goalieFactor or 0.88 --Goalie pos
  local factor1 = factor2 - 0.02 --max ball pos
  ballGlobal[1] = math.max(  -Config.world.xBoundary * factor1, ballGlobal[1])


  local goaliePosX = Config.goaliePosX or 0.08
  local t = (-Config.world.xBoundary+ goaliePosX - ballGlobal[1])/math.cos(ballGoalAngleCenter) 





  local target_position_x = -Config.world.xBoundary+ goaliePosX
--  local target_position_x = pose[1]

  local target_position_y = ballGlobal[2] + math.sin(ballGoalAngleCenter)*t
  local target_position_a = pose[3]

  local max_goalie_y = Config.max_goalie_y or 0.7

  target_position_y = math.max(
    -Config.world.goalWidth/2 * max_goalie_y, math.min(
      Config.world.goalWidth/2 * max_goalie_y , target_position_y* ballYFactor ))


  local goalie_y_ratio = math.abs(target_position_y / (Config.world.goalWidth/2))

  local x_diff_side = -0.20


  target_position_x = target_position_x + (x_diff_side*goalie_y_ratio)



print(target_position_x)

--  target_position_a = pose[3]
  target_position_a = 0 --this fixes drifting

  if Config.goalie_turn_to_ball then  --Facing towards the ball
    local ballfromtarget = {ballGlobal[1]-target_position_x,ballGlobal[2]-target_position_y}
    target_position_a = math.atan2(ballfromtarget[2],ballfromtarget[1])
  end

  return {target_position_x, target_position_y, target_position_a}
end

function robocupplanner.getVelocityGoalie(pose,target_pose , threshold)  
  -- Distance to the waypoint

  threshold = threshold or 0.2
  local homeRelative = util.pose_relative(target_pose,pose)
  local rHomeRelative = math.sqrt(homeRelative[1]^2+homeRelative[2]^2)
  local aHomeRelative = util.mod_angle(math.atan2(homeRelative[2],homeRelative[1]))
  local homeRot = math.abs(aHomeRelative)
  local homeRotBack = math.abs(util.mod_angle(aHomeRelative - math.pi))

  local goalie_threshold_x = Config.goalie_threshold_x or 0.10


  if math.abs(homeRelative[1])<goalie_threshold_x and 
    math.abs(homeRelative[2])<threshold and
    math.abs(util.mod_angle(homeRelative[2])) < 20*math.pi/180     then
    return {0,0,0},true
  end

  local vx,vy,va = 0,0,0

  va = 0.3*util.mod_angle(homeRelative[3])
  maxStep,maxA = 0.06,0.2
  vx = maxStep * homeRelative[1]/rHomeRelative
  vy = maxStep * homeRelative[2]/rHomeRelative
  va = math.min(maxA, math.max(-maxA, va))
  return{vx,vy,va},false
end





return robocupplanner
