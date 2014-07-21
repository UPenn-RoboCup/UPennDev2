-- A RoboCup World
local libWorld = {}

-- TODO: Add Attack bearing
-- TODO: Add Webots ground truth knowledge
local Body   = require'Body'
local vector = require'vector'
local util = require'util'
local ballFilter = require'ballFilter'
local poseFilter = require'poseFilter'
local obsFilter = require'obstacleFilter'
local odomScale = Config.world.odomScale
local use_imu_yaw = Config.world.use_imu_yaw
local RESAMPLE_PERIOD = Config.world.resample_period
local RESAMPLE_COUNT = Config.world.resample_count


require'wcm'
require'gcm'
require'mcm'

-- Timestamps
local t_entry
-- Cycle count
local count
-- Objects
local ball, goal, obstacle, line
-- Obstacle filters
local OF = {}

-- Initial odometry
local uOdometry0 = vector.zeros(3)
-- Save the resampling times
local t_resample = 0


local obstacles={}
local function reset_obstacles()
  obstacles={}
  wcm.set_obstacle_num(0)  
end

local function new_obstacle(a,r)
  --2nd order stat for the obstacle group
  return {asum=a,rsum=r,asqsum=a*a,rsqsum=r*r,count=1,aave=a,rave=r}
  --return {filter=obsFilter.new(a,r), count=1}
end

local function add_obstacle(a, r, da, dr)
  local min_dist = math.huge
  local min_index=0
  for i=1,#obstacles do
    --Just check angle to cluster observation
    --TODO: we can use distance too
    local adist = math.abs(util.mod_angle(obstacles[i].aave-a))
    --local adist = math.abs(util.mod_angle(obstacles[i].filter.a - a))
    if adist<min_dist then min_dist,min_index = adist,i end
  end
  if min_index==0 or min_dist>10*DEG_TO_RAD then 
    obstacles[#obstacles+1] = new_obstacle(a,r)
  else
   --obstacles[min_index].filter:observation_ra(r, a, dr, da)
   obstacles[min_index].count = obstacles[min_index].count + 1
    
---[[    
    obstacles[min_index].asum = obstacles[min_index].asum + a
    obstacles[min_index].rsum = obstacles[min_index].rsum + r
		-- TODO: these are not used for filtering
    obstacles[min_index].asqsum = obstacles[min_index].asqsum + a*a
    obstacles[min_index].rsqsum = obstacles[min_index].rsqsum + r*r
    obstacles[min_index].aave = obstacles[min_index].asum/obstacles[min_index].count
    obstacles[min_index].rave = obstacles[min_index].rsum/obstacles[min_index].count
--]]
  end
end

local function update_obstacles()  
  local counts={}
  for i=1,#obstacles do
--    print(string.format("Obstacle %d angle: %d dist: %.1f count: %d",
--      i,obstacles[i].aave*180/math.pi, obstacles[i].rave, obstacles[i].count))
    counts[i]=obstacles[i].count
  end
  --Sort the obstacles by their count
  table.sort(counts, function (a,b) return a>b end)
  --write top 3 obstacles to wcm
  local pose = wcm.get_robot_pose()
  for i=1, math.min(3,#obstacles) do
    local not_found,j = true,1
    while not_found and j< #obstacles+1 do
      if obstacles[j].count==counts[i] then
				--print('counts:',counts[i])
        local x = obstacles[j].rave * math.cos(obstacles[j].aave )
        local y = obstacles[j].rave * math.sin(obstacles[j].aave )
        
        --local x = obstacles[j].filter.r * math.cos(obstacles[j].filter.a)
        --local y = obstacles[j].filter.r * math.sin(obstacles[j].filter.a)
        
        local obs_global = util.pose_global({x,y,0},pose)
        wcm['set_obstacle_v'..i]({obs_global[1],obs_global[2]})
        not_found = false
      end
      j=j+1
    end
  end
  wcm.set_obstacle_num(math.min(2,#obstacles))
end


local yaw0 = 0
local function update_odometry(uOdometry)  
  -- Scale the odometry
  uOdometry[1] = odomScale[1] * uOdometry[1]
  uOdometry[2] = odomScale[2] * uOdometry[2]
  uOdometry[3] = odomScale[3] * uOdometry[3]
  -- Next, grab the gyro yaw

  if use_imu_yaw then    
    if IS_WEBOTS then
      gps_pose = wcm.get_robot_pose_gps()
      uOdometry[3] = gps_pose[3] - yaw0
      yaw0 = gps_pose[3]
    else
      local yaw = Body.get_rpy()[3]
      uOdometry[3] = yaw - yaw0
      yaw0 = yaw
    end
  end

  -- Update the filters based on the new odometry
  ballFilter.odometry(unpack(uOdometry))
  poseFilter.odometry(unpack(uOdometry))
end



local function sort_obs(t0, num)
  local function comp(x1,x2) return x1>x2 end 
  local t1, t2 = {}, {}
  for i=1,#t0 do t1[t0[i]] = i end
  table.sort(t0, comp)
  for i=1,num do t2[i] = t1[t0[i]] end
  return t2
end


local goal_type_to_filter = {
  -- Single unknown post
  [0] = poseFilter.post_unknown,
  -- Left
  [1] = poseFilter.post_left,
  -- Right
  [2] = poseFilter.post_right,
  -- Both
  [3] = poseFilter.post_both,
}

local function update_vision(detected)
  local t = unix.time()
  if t - t_resample > RESAMPLE_PERIOD or count%RESAMPLE_COUNT==0 then
    poseFilter.resample()
    if mcm.get_walk_ismoving()==1 then poseFilter.addNoise() end    
  end
  -- If the ball is detected
	ball = detected.ball
  if ball then
    ballFilter.observation_xy(ball.v[1], ball.v[2], ball.dr, ball.da, ball.t)
  end
        

  -- We cannot find the ball. 
  --add fake observation at behind the robot so that robot can start turning
  if wcm.get_ball_notvisible()==1 then
    wcm.set_ball_notvisible(0)
    if gcm.get_game_role()==1 then
      ballFilter.observation_xy(-0.5,0, 0.5, 20*math.pi/180, t)
    end
  end



--------------------------------------------------------------------------------
-- TODO: fix nan bug with this
  -- If the goal is detected
	goal = detected.posts
  if goal then
    if goal[1].type == 3 then
      if Config.debug.goalpost then
        print(string.format("Two post observation: type %d v1(%.2f %.2f) v2(%.2f %.2f)",
          goal[1].type,
          goal[1].v[1],goal[1].v[2],
          goal[2].v[1],goal[2].v[2]
          ))
      end
      if not Config.disable_goal_vision then
        goal_type_to_filter[goal[1].type]({goal[1].v, goal[2].v})
      end
    else
      if Config.debug.goalpost then
        print(string.format("Single post observation: type %d v(%.2f %.2f)",
          goal[1].type,
          goal[1].v[1],goal[1].v[2]        
          ))
      end
      if not Config.disable_goal_vision then
        goal_type_to_filter[goal[1].type]({goal[1].v, vector.zeros(4)})
      end
    end
  end
--------------------------------------------------------------------------------


  if wcm.get_obstacle_reset()==1 then
    reset_obstacles()
    wcm.set_obstacle_reset(0)
  end

  -- If the obstacle is detected
  obstacle = detected.obstacles
  if obstacle then
    --SJ: we keep polar coordinate statstics of the observed obstacles
    -- print("detected obstacles:",#obstacle.xs)
    for i=1,#obstacle.xs do
      local x, y = obstacle.xs[i], obstacle.ys[i]
      local r =math.sqrt(x^2+y^2)
      local a = math.atan2(y,x)
      --local dr, da = 0.1*r, 15*DEG_TO_RAD -- webots
      local dr, da = 0.1*r, 5*DEG_TO_RAD -- TODO
      add_obstacle(a,r, da,dr)
    end    
    update_obstacles()

  end  -- end of obstacle
  
end

function libWorld.entry()
  t_entry = unix.time()
  -- Initialize the pose filter
  -- poseFilter.initialize_unified()
  poseFilter.initialize()
  -- Save this resampling time
  t_resample = t_entry
  -- Set the initial odometry
  wcm.set_robot_pose({0,0,0})
  wcm.set_robot_odometry({0,0,0})
  wcm.set_robot_traj_num(0)
  wcm.set_obstacle_num(0)  
  wcm.set_ball_notvisible(0)
  -- Processing count
  count = 0
  
end

function libWorld.update(uOdom, detection)
  local t = unix.time()
  -- Run the updates
  if wcm.get_robot_reset_pose()==1 or (gcm.get_game_state()~=3 and gcm.get_game_state()~=6) then    
    if gcm.get_game_role()==0 then
      --Goalie initial pos
      local factor2 = 0.99
      poseFilter.initialize({-Config.world.xBoundary*factor2,0,0},{0,0,0})
      wcm.set_robot_pose({-Config.world.xBoundary*factor2,0,0})
      wcm.set_robot_odometry({-Config.world.xBoundary*factor2,0,0})
    else --Attacker initial pos
      poseFilter.initialize({0,0,0},{0,0,0})
      wcm.set_robot_pose({0,0,0})
      wcm.set_robot_odometry({0,0,0})
    end

    if use_imu_yaw then    
      if IS_WEBOTS then
        gps_pose = wcm.get_robot_pose_gps()
        yaw0 = gps_pose[3]
      else
        local yaw = Body.get_rpy()[3]
        yaw0 = yaw
      end
    end
  else
    update_odometry(uOdom)
  end

  update_vision(detection)
  if Config.use_gps_pose then
    wcm.set_robot_pose(wcm.get_robot_pose_gps())
    wcm.set_obstacle_num(2)
    wcm.set_obstacle_v1(wcm.get_robot_gpsobs1())
    wcm.set_obstacle_v2(wcm.get_robot_gpsobs2())

    local pose = wcm.get_robot_pose_gps()
    local ballglobal = wcm.get_robot_gpsball()
    wcm.set_robot_ballglobal(ballglobal)

    local balllocal = util.pose_relative({ballglobal[1],ballglobal[2],0},pose)
    wcm.set_ball_x(balllocal[1])
    wcm.set_ball_y(balllocal[2])
    wcm.set_ball_t(Body.get_time())
  else

    local pose =wcm.get_robot_pose()
    local ball_x = wcm.get_ball_x()
    local ball_y = wcm.get_ball_y()

    local ballglobal = util.pose_global({ball_x,ball_y,0},pose)

    wcm.set_robot_ballglobal(ballglobal)
    -- Update pose in wcm
    wcm.set_robot_pose(vector.pose{poseFilter.get_pose()})
  end



  
  -- Increment the process count
  count = count + 1
end

local game_state_names={
  "Initial",
  "Ready",
  "Set",
  "Playing",
  "Finished",
  "Untorqued",
  "Testing"
}

local motion_state_names={
  "Idle",
  "Init",
  "Stance",
  "WalkInit",
  "Walk",
  "WalkEnd",
  "WalkKick"
}


function libWorld.send()
  local to_send = {}
  to_send.info = ''
  -- Robot info
  to_send.pose = vector.new(wcm.get_robot_pose())

  if IS_WEBOTS then
    to_send.gpspose = wcm.get_robot_pose_gps()
    to_send.gpsball = wcm.get_robot_gpsball()
    to_send.gpsobs1 = wcm.get_robot_gpsobs1()
    to_send.gpsobs2 = wcm.get_robot_gpsobs2()
  end

  to_send.role = gcm.get_game_role()
  to_send.time = Body.get_time()
  

  if gcm.get_game_role()==0 then
    to_send.info=to_send.info.."Role: Goalie\n"
  elseif gcm.get_game_role()==1 then
    to_send.info=to_send.info.."Role: ATTACKER\n"
  else
    to_send.info=to_send.info.."Role: IDLE\n"
  end

  to_send.gamestate = gcm.get_game_state()
  to_send.info=to_send.info.."Game :"..game_state_names[to_send.gamestate+1]
  to_send.info=to_send.info.."\n"

  to_send.motionstate = mcm.get_motion_state()
  to_send.info=to_send.info.."Motion :"..motion_state_names[to_send.motionstate +1]
  to_send.info=to_send.info.."\n"



  to_send.info = to_send.info..string.format(
    'Pose: %.2f %.2f (%.1f)\n', to_send.pose[1], to_send.pose[2], to_send.pose[3]*RAD_TO_DEG)

  

  to_send.role = gcm.get_game_role()
  to_send.time = Body.get_time()
  
  -- Ball info
  if ball then
    to_send.ball = {}
    to_send.ball.x = wcm.get_ball_x()
    to_send.ball.y = wcm.get_ball_y()
    to_send.ball.t = wcm.get_ball_t()
    to_send.info = to_send.info..string.format(
      'Ball: %.2f %.2f\n', to_send.ball.x, to_send.ball.y)
  end

  -- Goal info
  if goal then
    to_send.goal = {}
    to_send.goal.type = goal[1].type
    to_send.info = to_send.info..string.format(
      'Post type: %d \n', goal[1].type )
    
    to_send.goal.v1 = goal[1].v
    to_send.info = to_send.info..string.format(
      'Post1: %.2f %.2f\n', to_send.goal.v1[1], to_send.goal.v1[2])
    
    if goal[1].type==3 then
      to_send.goal.v2 = goal[2].v
      to_send.info = to_send.info..string.format(
        'Post2: %.2f %.2f\n', to_send.goal.v2[1], to_send.goal.v2[2])
    end
  end  
    
  -- Obstacles
  if wcm.get_obstacle_num()>0 then
    local obs = {}
    for i=1,wcm.get_obstacle_num() do
      obs[i] = wcm['get_obstacle_v'..i]()
      to_send.info = to_send.info..string.format(
        'Obstacle: %.2f %.2f\n', unpack(obs[i]) )
    end
    to_send.obstacle = obs
  else
    --WE HAVE TO CLEAR OBSTACLE
    local obs = {}
    to_send.obstacle = obs
  end
  
  -- Lines
  if line and gcm.get_game_role()==0 then
    wcm.set_line_detect(1)
  end

  --TRAJECTORY INFO
  local traj = {}
  traj.num = wcm.get_robot_traj_num()
  traj.x = wcm.get_robot_trajx()
  traj.y = wcm.get_robot_trajy()
  

  traj.goalto = wcm.get_robot_goalto()
  traj.goal1 = wcm.get_robot_goal1()
  traj.goal2 = wcm.get_robot_goal2()

  traj.ballglobal = wcm.get_robot_ballglobal()  

  traj.kickneeded = wcm.get_robot_kickneeded()
  traj.kickangle1 = wcm.get_robot_kickangle1()
  traj.kickangle2 = wcm.get_robot_kickangle2()
  traj.ballglobal2 = wcm.get_robot_ballglobal2() 
  traj.ballglobal3 = wcm.get_robot_ballglobal3()


  traj.goalangles={}

  to_send.traj = traj


  return to_send
end

function libWorld.exit()
end

function libWorld.get_pose()
  return vector.pose{poseFilter.get_pose()}
end

libWorld.update_odometry = update_odometry
libWorld.update_vision = update_vision

return libWorld
