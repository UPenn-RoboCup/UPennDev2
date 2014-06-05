local Body = require'Body'
local vector = require'vector'
local util = require'util'
local ballFilter = require'ballFilter'
local poseFilter = require'poseFilter'

require'wcm'
require'vcm'
require'gcm'
require'mcm'

local world = {}


--SJ: Velocity filter is always on
--We can toggle whether to use velocity to update ball position estimate

mod_angle = util.mod_angle

-- local Velocity = require('Velocity')  

--Are we using same colored goals?
use_same_colored_goal = Config.world.use_same_colored_goal or 0
--Use ground truth pose and ball information for webots?
use_gps_only = Config.use_gps_only or 0
gps_enable = Body.gps_enable or 0



--For NSL, eye LED is not allowed during match
led_on = Config.led_on or 1 --Default is ON

-- ball = {}
-- ball.t = 0  --Detection time
-- ball.x = 1.0
-- ball.y = 0
-- ball.vx = 0
-- ball.vy = 0
-- ball.p = 0 

pose = {}
pose.x = 0
pose.y = 0
pose.a = 0

uOdometry0 = vector.new({0, 0, 0})
count = 0
cResample = Config.world.cResample 


odomScale = Config.walk.odomScale or Config.world.odomScale

--SJ: they are for IMU based navigation
imuYaw = Config.world.imuYaw or 0
yaw0 = 0

--Track gcm state
gameState = 0

-- Debugging info
local debug_str
local function add_debug_msg(str)
  debug_str = debug_str..str
end


function world.init_particles()
  -- if wcm.get_robot_role()==1 then  -- attacker
  --   poseFilter.initialize(nil, nil)  
  -- elseif wcm.get_robot_role()==2  then-- defender
  --   poseFilter.initialize({postFilter.postYellow[1][1], 0, math.pi})
  -- end
  
  poseFilter.initialize_unified(nil, nil)
  --TODO: else unknown
end

function world.entry()
  count = 0
  world.init_particles()
  -- Velocity.entry()
end

-- -- Not in need for adult size
-- function world.init_particles_manual_placement()
--   if gcm.get_team_role() == 0 then
--   -- goalie initialized to different place
--     goalDefend= world.get_goal_defend()
--     util.ptable(goalDefend)
--     dp = vector.new({0.04,0.04,math.pi/8})
--     if goalDefend[1] > 0 then
--       poseFilter.initialize(vector.new({goalDefend[1],0,math.pi}), dp)
--     else
--       poseFilter.initialize(vector.new({goalDefend[1],0,0}), dp)
--     end
--   else
--     poseFilter.initialize_manual_placement()
--     if (useSoundLocalization > 0) then
--       SoundFilter.reset()
--     end
--   end
-- end

-- 
-- function world.allLessThanTenth(table)
--   for k,v in pairs(table) do
--     if v >= .1 then
--       return false
--     end
--   end
--   return true
-- end
-- 
-- function world.allZeros(table)
--   for k,v in pairs(table) do
--     if v~=0 then
--       return false
--     end
--   end
--   return true
-- end


function world.update_odometry()
  debug_str = ''

  count = count + 1
  uOdometry = Body.get_odometry()

  uOdometry[1] = odomScale[1]*uOdometry[1]
  uOdometry[2] = odomScale[2]*uOdometry[2]
  uOdometry[3] = odomScale[3]*uOdometry[3]*Body.DEG_TO_RAD

  add_debug_msg(string.format('Odom_dx: %.2f  Odom_dy: %.2f Odom_da: %.2f\n', 
    uOdometry[1], uOdometry[2], uOdometry[3]))

  --Gyro integration based IMU
  if imuYaw==1 then
    yaw = Body.get_sensor_imuAngle(3)
    uOdometry[3] = yaw-yaw0
    yaw0 = yaw
    --print("Body yaw:",yaw*180/math.pi, " Pose yaw ",pose.a*180/math.pi)
  end

  ballFilter.odometry(uOdometry[1], uOdometry[2], uOdometry[3])
  poseFilter.odometry(uOdometry[1], uOdometry[2], uOdometry[3])
end


function world.update_pos()
  -- update localization without vision (for odometry testing)
  if count % cResample == 0 then
    poseFilter.resample()
  end

  pose.x,pose.y,pose.a = poseFilter.get_pose()
  update_shm()
end

-- TODO: vcm or called through reference?
function world.update_vision(ball, goal, line)
  debug_str = ''
  
  --update ground truth
	if gps_enable>0 then
    gps_pose0 = Body.get_sensor_gps()
    --GPS is attached at torso, so we should discount body offset
    uBodyOffset = mcm.get_walk_bodyOffset()
    gps_pose = util.pose_global(-uBodyOffset,gps_pose0)

    gps_pose_xya={}
    gps_pose_xya.x=gps_pose[1]
    gps_pose_xya.y=gps_pose[2]
    gps_pose_xya.a=gps_pose[3]
    gps_attackBearing = world.get_attack_bearing_pose(gps_pose_xya)

    wcm.set_robot_pose_gps(gps_pose)
    wcm.set_robot_gps_attackbearing(gps_attackBearing)
  else
    wcm.set_robot_pose_gps({pose.x,pose.y,pose.a})
    -- wcm.set_robot_gps_attackbearing(world.get_attack_bearing())
  end

  if use_gps_only>0 then
    --Use GPS pose instead of using particle filter
    pose.x,pose.y,pose.a=gps_pose[1],gps_pose[2],gps_pose[3]
    --Use GPS ball pose instead of ball filter
    ballGlobal=wcm.get_robot_gps_ball()    
    ballLocal = util.pose_relative(ballGlobal,gps_pose)
    ball.x, ball.y = ballLocal[1],ballLocal[2]
    wcm.set_ball_v_inf({ball.x,ball.y}) --for bodyAnticipate

    ball_gamma = 0.3
    if vcm.get_ball_detect()==1 then
      ball.p = (1-ball_gamma)*ball.p+ball_gamma
      ball.t = Body.get_time()
      -- Update the velocity
      Velocity.update(ball.x,ball.y)
      ball.vx, ball.vy, dodge  = Velocity.getVelocity()
    else
      ball.p = (1-ball_gamma)*ball.p
      Velocity.update_noball()--notify that ball is missing
    end
    update_shm()

    return
  end

  -- resample?
  WEBOTS = true
  WEBOTS = false
  if count % cResample == 0 then
    poseFilter.resample()
    if not WEBOTS then
      poseFilter.addNoise()
    end
  end

  -- Reset heading if robot is down
  -- if (mcm.get_walk_isFallDown() == 1) then
  --   poseFilter.reset_heading()
  -- 
  --   if (useSoundLocalization > 0) then
  --     SoundFilter.reset()
  --   end
  -- end

    
  -- ball
  ball_gamma = 0.3
  if (ball.detect == 1) then
    -- ball.t = Body.get_time()
    -- TODO: use ball.t or Body.get_time()??
    ballFilter.observation_xy(ball.v[1],ball.v[2],ball.dr,ball.da, ball.t)
    
    -- ball.p = (1-ball_gamma)*ball.p+ball_gamma

    -- Update the velocity
--    Velocity.update(v[1],v[2])

    -- use centroid info only
    -- ball_v_inf = wcm.get_ball_v_inf()
    -- Velocity.update(ball_v_inf[1],ball_v_inf[2])
    -- ball.vx, ball.vy, dodge  = Velocity.getVelocity()
  else
    -- ball.p = (1-ball_gamma)*ball.p
    -- Velocity.update_noball()--notify that ball is missing
  end


  if goal.detect == 1 then
    --TODO:
    wcm.set_goal_t(goal.t)
    wcm.set_goal_t(Body.get_time())

    if (goal.type == 0) then
    	add_debug_msg('Single unknown post...\n')
      poseFilter.post_unknown(goal.v)
    elseif(goal.type == 1) then
    	add_debug_msg('Left post...\n')
      poseFilter.post_left(goal.v)
    elseif(goal.type == 2) then
    	add_debug_msg('Right post...\n')
      poseFilter.post_right(goal.v)
    elseif(goal.type == 3) then
    	add_debug_msg('Both posts...\n')
      poseFilter.post_both(goal.v)
    end
  
  end



  -- TODO: can we make sure that we use the longest one?
  -- if line.detect == 1 then
  --   local v = vcm.get_line_v()
  --   local a = vcm.get_line_angle()
  -- 
  --   poseFilter.line(v, a)--use longest line in the view
  -- end
  
  -- if vcm.get_corner_detect() == 1 then
  --   local v=vcm.get_corner_v()
  --   poseFilter.corner(v)
  -- end
  
  -- if vcm.get_landmark_detect() == 1 then
  --   local color = vcm.get_landmark_color()
  --   local v = vcm.get_landmark_v()
  --   if color == Config.color.yellow then
  --       poseFilter.landmark_yellow(v)
  --   else
  --       poseFilter.landmark_cyan(v)
  --   end
  -- else
  --   if vcm.get_goal_detect() == 0 then
  --     goal_led={0,0,0}
  --   end
  -- end

  pose.x,pose.y,pose.a = poseFilter.get_pose()
  
  local pose_odom = wcm.get_robot_pose()
  add_debug_msg(string.format('Pose odom: %.2f %.2f %.2f\n', 
    pose_odom[1], pose_odom[2], pose_odom[3]))
  add_debug_msg(string.format('Pose vision: %.2f %.2f %.2f\n', 
    pose.x, pose.y, pose.a))
    
  -- world.update_led()
  world.update_shm()
  
  return debug_str
end



-- function world.update_led()
--   --Turn on the eye light according to team color
--   --If gamecontroller is down
--   if gcm.get_game_state()~=3 and
--      gcm.get_game_gc_latency() > 10.0 then
-- 
--     if gcm.get_team_color() == 0 then --Blue team
--       Body.set_indicator_goal({0,0,0})
--       Body.set_indicator_ball({0,0,1})
--     else --Red team
--       Body.set_indicator_goal({0,0,0})
--       Body.set_indicator_ball({0,0,1})
--     end
--     return
--   end
-- 
--   --Only disable eye LED during playing
-- --  if led_on>0 and gcm.get_game_state()~=3 then
--   if led_on>0 then
--     Body.set_indicator_goal(goal_led)
--     Body.set_indicator_ball(ball_led)
--   else
--     Body.set_indicator_goal({0,0,0})
--     Body.set_indicator_ball({0,0,0})
--   end
-- end



function world.update_shm()
  -- update shm values

  wcm.set_robot_pose_vision({pose.x, pose.y, pose.a})
  wcm.set_robot_time(Body.get_time())

  -- wcm.set_ball_velx(ball.vx)
  -- wcm.set_ball_vely(ball.vy)
  -- wcm.set_ball_p(ball.p)

  -- wcm.set_goal_attack(world.get_goal_attack())
  -- wcm.set_goal_defend(world.get_goal_defend())
  -- wcm.set_goal_attack_bearing(world.get_attack_bearing())
  wcm.set_goal_attack_angle(world.get_attack_angle())
  wcm.set_goal_defend_angle(world.get_defend_angle())

  -- wcm.set_goal_attack_post1(world.get_attack_posts()[1])
  -- wcm.set_goal_attack_post2(world.get_attack_posts()[2])

  -- wcm.set_robot_is_fall_down(mcm.get_walk_isFallDown())
  
  --Particle information
  wcm.set_particle_x(poseFilter.xp)
  wcm.set_particle_y(poseFilter.yp)
  wcm.set_particle_a(poseFilter.ap)
  wcm.set_particle_w(poseFilter.wp)

end

function world.exit()
end


function world.get_ball()
  return ball
end

function world.get_pose()
  return pose
end

function world.zero_pose()
  poseFilter.zero_pose()
end

function world.get_attack_bearing()
  return world.get_attack_bearing_pose(pose)
end

--Get attack bearing from pose0
function world.get_attack_bearing_pose(pose0)
  if gcm.get_team_color() == 1 then
    -- red attacks cyan goal
    postAttack = poseFilter.postCyan
  else
    -- blue attack yellow goal
    postAttack = poseFilter.postYellow
  end
  -- make sure not to shoot back towards defensive goal:
  local xPose = math.min(math.max(pose0.x, -0.99*poseFilter.xLineBoundary),
                          0.99*poseFilter.xLineBoundary)
  local yPose = pose0.y
  local aPost = {}
  aPost[1] = math.atan2(postAttack[1][2]-yPose, postAttack[1][1]-xPose)
  aPost[2] = math.atan2(postAttack[2][2]-yPose, postAttack[2][1]-xPose)
  local daPost = math.abs(poseFilter.mod_angle(aPost[1]-aPost[2]))
  attackHeading = aPost[2] + .5*daPost
  attackBearing = poseFilter.mod_angle(attackHeading - pose0.a)

  return attackBearing, daPost
end


function world.get_attack_angle()
  goalAttack = {Config.world.goalUpper[1][1], 0, 0}

  dx = goalAttack[1] - pose.x
  dy = goalAttack[2] - pose.y
  return mod_angle(math.atan2(dy, dx) - pose.a)
end

function world.get_defend_angle()
  goalDefend = {Config.world.goalLower[1][1], 0, 0}

  dx = goalDefend[1] - pose.x
  dy = goalDefend[2] - pose.y
  return mod_angle(math.atan2(dy, dx) - pose.a)
end


return world
