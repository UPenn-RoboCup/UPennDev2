local libWorld = {}

--SJ: a dummy world file
--we can add slam, odometry, and so on later

-- TODO: Add Attack bearing
-- TODO: Add Webots ground truth knowledge
local Body   = require'Body'
local vector = require'vector'
local util = require'util'
local odomScale = Config.world.odomScale
local odomDrift = Config.world.odomDrift or 0

local ballFilter = require'ballFilter'
local poseFilter = require'poseFilter'
local RESAMPLE_PERIOD = Config.world.resample_period
local RESAMPLE_COUNT = Config.world.resample_count

require'wcm'
require'gcm'
require'mcm'
wcm.set_robot_use_imu_yaw(Config.world.use_imu_yaw and 1 or 0)

-- Timestamps
local t_entry
-- Cycle count
local count
-- Objects
local ball, goal, obstacle, line
-- Obstacle filters
local OF = {}


local t_old

-- Initial odometry
local uOdometry0 = vector.zeros(3)
-- Save the resampling times
local t_resample = 0

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

local obstacles={}
local function reset_obstacles()
  obstacles={}
  wcm.set_obstacle_num(0)
end

local function new_obstacle(a,r)
  --2nd order stat for the obstacle group
  return {asum=a,rsum=r,asqsum=a*a,rsqsum=r*r,count=1,aave=a,rave=r}
end

local function add_obstacle(a, r, da, dr)
  local min_dist = math.huge
  local min_index=0
  for i=1,#obstacles do
    --Just check angle to cluster observation
    --TODO: we can use distance too
    local adist = math.abs(util.mod_angle(obstacles[i].aave-a))
    if adist<min_dist then min_dist,min_index = adist,i end
  end
  if min_index==0 or min_dist>10*DEG_TO_RAD then
    obstacles[#obstacles+1] = new_obstacle(a,r)
  else
    obstacles[min_index].count = obstacles[min_index].count + 1
    obstacles[min_index].asum = obstacles[min_index].asum + a
    obstacles[min_index].rsum = obstacles[min_index].rsum + r
		-- TODO: these are not used for filtering
    obstacles[min_index].asqsum = obstacles[min_index].asqsum + a*a
    obstacles[min_index].rsqsum = obstacles[min_index].rsqsum + r*r
    obstacles[min_index].aave = obstacles[min_index].asum/obstacles[min_index].count
    obstacles[min_index].rave = obstacles[min_index].rsum/obstacles[min_index].count
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

  --print('dOdom', unpack(uOdometry))


--t_old = t_entry

  local t = Body.get_time()
  local t_passed = t-t_old
  t_old=t

  uOdometry[1] = odomScale[1] * uOdometry[1] + t_passed * wcm.get_robot_odomfactor()/0.80
  uOdometry[2] = odomScale[2] * uOdometry[2]
  uOdometry[3] = odomScale[3] * uOdometry[3]
  -- Next, grab the gyro yaw

--  if Config.use_imu_yaw and mcm.get_walk_ismoving()>0 then



  if Config.use_imu_yaw then
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



  --Update pose using odometry info for now
  local pose = wcm.get_robot_pose()


  if IS_WEBOTS and Config.world.use_gps_yaw then


    local old_pose = wcm.get_robot_pose()
    local gps_pose = wcm.get_robot_pose_gps()
    uOdometry[3]=gps_pose[3]-old_pose[3]

    --print(old_pose[3]*180/math.pi, gps_pose[3]*180/math.pi)
  end


  -- Update the filters based on the new odometry
  ballFilter.odometry(unpack(uOdometry))
  poseFilter.odometry(unpack(uOdometry))

  --TODO: slam or wall detection-based pose

  local new_pose = util.pose_global(uOdometry, pose)




  return new_pose

end

local function update_vision(detected)
  if not detected then return end

  local t = Body.get_time()
  if t - t_resample > RESAMPLE_PERIOD or count%RESAMPLE_COUNT==0 then
    poseFilter.resample()
    if mcm.get_walk_ismoving()==1 then poseFilter.addNoise() end
  end

  -- If the ball is detected on this frame
	ball = detected.ball
  if Config.disable_ball_when_lookup and wcm.get_ball_disable()>0 then
--    print("BALL DISABLED")
  elseif ball then
      ballFilter.observation_xy(ball.v[1], ball.v[2], ball.dr, ball.da, ball.t)
  end

  -- We cannot find the ball.
  -- Add fake observation at behind the robot so that robot can start turning
  if wcm.get_ball_notvisible()==1 then
    wcm.set_ball_notvisible(0)
    if gcm.get_game_role()==1 and Config.robot_turnaround then
      ballFilter.observation_xy(-0.5,0, 0.5, 20*DEG_TO_RAD, t)
    end
  end

--------------------------------------------------------------------------------
  -- TODO: fix nan bug with this
  -- If the goal is detected

  if wcm.get_goal_disable()==0 then --now disable goal detection unless we look up
  	goal = detected.posts
    if type(goal)=='table' and #goal>0 then
      if goal[1].type == 3 then
        if Config.debug.goalpost then
          print(string.format(
            "Two post observation: type %d v1(%.2f %.2f) v2(%.2f %.2f)",
            goal[1].type,
            goal[1].v[1],goal[1].v[2],
            goal[2].v[1],goal[2].v[2]
            ))
        end
        if (not Config.disable_goal_vision) then
          if Config.goalie_odometry_only and gcm.get_game_role()==0 then
--            print("Goalie, goal update disabled")
          else
            goal_type_to_filter[goal[1].type]({goal[1].v, goal[2].v})
          end
        end
      else
        if Config.debug.goalpost then
          print(string.format("Single post observation: type %d v(%.2f %.2f)",
            goal[1].type,
            goal[1].v[1],goal[1].v[2]
            ))
        end
        if not Config.disable_goal_vision then
          if Config.goalie_odometry_only and gcm.get_game_role()==0 then
--            print("Goalie, goal update disabled")
          else
            goal_type_to_filter[goal[1].type]({goal[1].v, vector.zeros(4)})
          end
        end
      end
    end
  else
    goal=nil
  end
  ------------------------------------------------------------------------------


  if wcm.get_obstacle_reset()==1 then
    reset_obstacles()
    wcm.set_obstacle_reset(0)
  end

  if wcm.get_obstacle_enable()==1 then
    -- If the obstacle is detected
    obstacle = detected.obstacles
    if obstacle then
      --SJ: we keep polar coordinate statstics of the observed obstacles
      -- print("detected obstacles:",#obstacle.xs)
      for i=1,#obstacle.xs do
        local x, y = obstacle.xs[i], obstacle.ys[i]
        local r = math.sqrt(x^2+y^2)
        local a = math.atan2(y,x)
        --local dr, da = 0.1*r, 15*DEG_TO_RAD -- webots
        local dr, da = 0.1*r, 5*DEG_TO_RAD -- TODO
        add_obstacle(a,r, da,dr)
      end
      update_obstacles()

    end  -- end of obstacle
  end

  if type(detected.line)=='table' then
    --print()
    for i, v in ipairs(detected.line.v) do
      local v_close = (vector.new(v[1]) + vector.new(v[2])) / 2
      local a = detected.line.angle[i]
      --print('Line Angle', a*RAD_TO_DEG)
      --print('Local Line Position', unpack(v_close))
    end
    --poseFilter.line_observation(v, a)
  end

end

function libWorld.pose_reset(pose0)
  print("libWorld | POSE RESET!")
  wcm.set_robot_reset_pose(0)
  wcm.set_robot_pose(pose0 or {0,0,0})
  wcm.set_robot_odometry({0,0,0})


print("yaw0:",yaw0*180/math.pi)
print("yaw:",Body.get_rpy()[3]*180/math.pi)

  yaw0 = Body.get_rpy()[3]
  poseFilter.initialize(pose0 or {0,0,0},{0,0,0})
  if IS_WEBOTS then
    gps_pose = wcm.get_robot_pose_gps()
    yaw0 = gps_pose[3]
    wcm.set_robot_pose_gps0(wcm.get_robot_pose_gps())
  end
end

local function print_pose()
  if not Config.debug.world then return end
  local pose = wcm.get_robot_pose()
  local gpspose1 = wcm.get_robot_pose_gps()
  local gpspose0 = wcm.get_robot_pose_gps0()
  local gpspose = util.pose_relative(gpspose1,gpspose0)
  print(string.format(
    "pose: %.3f %.3f %d gps: %.3f %.3f %d",
    pose[1],pose[2],pose[3]*RAD_TO_DEG,
    gpspose[1],gpspose[2],gpspose[3]*180/math.pi))
  local uTorso = mcm.get_status_uTorso()
  print("libWorld | uTorso:",unpack(uTorso))
end

function libWorld.entry()
	wcm.set_robot_use_imu_yaw(Config.world.use_imu_yaw and 1 or 0)
	t_entry = Body.get_time()
  -- Initialize the pose filter
  -- poseFilter.initialize_unified()
  poseFilter.initialize()
  -- Save this resampling time
  t_resample = t_entry
  t_old = t_entry
  -- Set the initial odometry
  wcm.set_robot_pose({0,0,0})
  wcm.set_robot_odometry({0,0,0})
  wcm.set_robot_traj_num(0)
  wcm.set_obstacle_num(0)
  wcm.set_ball_notvisible(0)

  reset_obstacles()

  libWorld.pose_reset()
  -- Processing count
  count = 0
end


local t_last_update=0

function libWorld.update(uOdom, detection)
  local t = Body.get_time()
--[[
  if t-t_last_update>2.0 then
    local pose =  wcm.get_robot_pose()
    print("Current pose:",pose[1],pose[2],pose[3]*RAD_TO_DEG)
    t_last_update=t
  end
--]]


  -- Run the updates
  if wcm.get_robot_reset_pose()==1 then
    libWorld.pose_reset()
    --This is called by motionInit
    --after init, reset pose to 0,0,0
  end

  local updated_pose
  if IS_WEBOTS and Config.use_gps_pose then
    local gpspose1 = wcm.get_robot_pose_gps()
    local gpspose0 = wcm.get_robot_pose_gps0()
    local gpspose = util.pose_relative(gpspose1,gpspose0)

    --subtract automatic compensation
    comoffset = mcm.get_stance_COMoffset()
    comoffset[3]=0
    gpspose = util.pose_global(comoffset, gpspose)
    updated_pose = vector.copy(gpspose)

    print_pose()
--  elseif wcm.get_robot_reset_pose()==1
    --or
--

  elseif (gcm.get_game_state()~=3 and gcm.get_game_state()~=6) then

    --Keep resetting pose unless the game state is playing or testing
    if gcm.get_game_role()==0 then
--     print("Goalie pose reset!")
      -- Goalie initial pos
      local factor2 = 0.99
      poseFilter.initialize({-Config.world.xBoundary*factor2,0,0},{0,0,0})
      updated_pose = {-Config.world.xBoundary*factor2,0,0}
      wcm.set_robot_odometry({-Config.world.xBoundary*factor2,0,0})

    else --Attacker initial pos
--      print("Attacker pose reset!")
      poseFilter.initialize({0,0,0},{0,0,0})
      wcm.set_robot_odometry({0,0,0})
      updated_pose = {0,0,0}
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
    print_pose()
    updated_pose = update_odometry(uOdom)
  end

  --to fix yaw jump
  if (gcm.get_game_state()~=3 and gcm.get_game_state()~=6) then
    local yaw = Body.get_rpy()[3]
    yaw0 = yaw
  end

  wcm.set_robot_pose(updated_pose)
  update_vision(detection)

  if IS_WEBOTS and Config.use_gps_vision then
    wcm.set_obstacle_num(2)
    wcm.set_obstacle_v1(wcm.get_robot_gpsobs1())
    wcm.set_obstacle_v2(wcm.get_robot_gpsobs2())
    local ballglobal = wcm.get_robot_gpsball()
    wcm.set_robot_ballglobal(ballglobal)
    local pose = wcm.get_robot_pose_gps()
    local balllocal = util.pose_relative(
      {ballglobal[1],ballglobal[2],0},
      pose
    )
    wcm.set_ball_x(balllocal[1])
    wcm.set_ball_y(balllocal[2])
    wcm.set_ball_t(Body.get_time())
--    print("global ball:",unpack(ballglobal))
  else

    -- Local ball
    local ball_x = wcm.get_ball_x()
    local ball_y = wcm.get_ball_y()

    -- Global ball
    local ballglobal = util.pose_global({ball_x,ball_y,0}, updated_pose)
    wcm.set_robot_ballglobal(ballglobal)
    if not (IS_WEBOTS and Config.use_gps_pose) then
      -- Update pose in wcm
      wcm.set_robot_pose(vector.pose{poseFilter.get_pose()})
    end
  end

  -- Increment the process count
  count = count + 1
end


function libWorld.send()

  -- Formatted data to send
  local info = {}
  local to_send = {
    time = Body.get_time(),
  }

  -- Pose
  local pose = wcm.get_robot_pose()
  to_send.pose = pose
  table.insert(info, string.format(
    'Pose: %.2f %.2f (%.1f)\n',
    pose[1], pose[2], pose[3]*RAD_TO_DEG
  ))

  -- Show the role
  local role = gcm.get_game_role()
  if role==0 then
    table.insert(info, "Role: Goalie")
  elseif role==1 then
    table.insert(info, "Role: Attacker")
  else
    table.insert(info, "Role: Idle")
  end
  to_send.role = role

  -- Game
  to_send.gamestate = gcm.get_game_state()
  table.insert(info, "Game: "..game_state_names[to_send.gamestate+1])

  -- TODO: Motion
  --to_send.motionstate = mcm.get_motion_state()
  --table.insert(info, "Motion: "..motion_state_names[to_send.motionstate+1])

  -- Ball info
  if ball then
    to_send.ball = {
      x = wcm.get_ball_x(),
      y = wcm.get_ball_y(),
      t = wcm.get_ball_t(),
    }
    table.insert(info, string.format(
      'Ball: %.2f %.2f', to_send.ball.x, to_send.ball.y)
    )
  end

  -- Obstacles
  if wcm.get_obstacle_num()>0 then
    local obs = {}
    for i=1,wcm.get_obstacle_num() do
      obs[i] = wcm['get_obstacle_v'..i]()
      table.insert(info,string.format(
        'Obstacle: %.2f %.2f', unpack(obs[i]))
      )
    end
    to_send.obstacle = obs
  else
    -- TODO: WE HAVE TO CLEAR OBSTACLE
    to_send.obstacle = {}
  end

  -- TODO: Lines
  if line and gcm.get_game_role()==0 then
    wcm.set_line_detect(1)
  end

  -- Goal info
  if type(goal)=='table' and #goal>0 then
    to_send.goal = {
      type = goal[1].type,
      v1 = goal[1].v
    }
    table.insert(info, string.format(
      'Post type: %d \n', goal[1].type )
    )
    table.insert(info, string.format(
      'Post1: %.2f %.2f\n', to_send.goal.v1[1], to_send.goal.v1[2])
    )
    if goal[1].type==3 then
      to_send.goal.v2 = goal[2].v
      table.insert(info, string.format(
        'Post2: %.2f %.2f\n', to_send.goal.v2[1], to_send.goal.v2[2])
      )
    end
  end

  --TRAJECTORY INFO
  local traj = {
    num = wcm.get_robot_traj_num(),
    x = wcm.get_robot_trajx(),
    y = wcm.get_robot_trajy(),

    goalto = wcm.get_robot_goalto(),
    goal1 = wcm.get_robot_goal1(),
    goal2 = wcm.get_robot_goal2(),

    ballglobal = wcm.get_robot_ballglobal()  ,

    kickneeded = wcm.get_robot_kickneeded(),
    kickangle1 = wcm.get_robot_kickangle1(),
    kickangle2 = wcm.get_robot_kickangle2(),
    ballglobal2 = wcm.get_robot_ballglobal2() ,
    ballglobal3 = wcm.get_robot_ballglobal3(),

    goalangles={},
  }

  to_send.traj = traj

  -- Format the info
  to_send.info = table.concat(info, '\n')
  return to_send
end

function libWorld.exit()
end

function libWorld.get_pose()
--TODO
--  return wcm.get_robot_pose(wcm.get_robot_pose_gps())
  --return vector.pose({0,0,0})
  --return vector.pose{poseFilter.get_pose()}
  return wcm.get_robot_pose()
end

libWorld.update_odometry = update_odometry
libWorld.update_vision = update_vision

return libWorld
