-- A RoboCup World
local libWorld = {}

-- TODO: Add Attack bearing
-- TODO: Add Webots ground truth knowledge
local Body   = require'Body'
local vector = require'vector'
local ballFilter = require'ballFilter'
local poseFilter = require'poseFilter'
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
local ball, goal, obstacle

-- Initial odometry
local uOdometry0 = vector.zeros(3)
-- Save the resampling times
local t_resample = 0


--WHY IS THIS NOT WORKING ANYMORE?
--local yaw0 = Body.get_sensor_rpy()[3]


local function update_odometry(uOdometry)
  -- Scale the odometry
  uOdometry[1] = odomScale[1] * uOdometry[1]
  uOdometry[2] = odomScale[2] * uOdometry[2]
  uOdometry[3] = odomScale[3] * uOdometry[3]
  -- Next, grab the gyro yaw
--[[
  if use_imu_yaw then    
    local yaw = Body.get_sensor_rpy()[3]
    uOdometry[3] = yaw - yaw0
    yaw0 = yaw
  end
--]]  
  -- Update the filters based on the new odometry
  ballFilter.odometry(unpack(uOdometry))
  poseFilter.odometry(unpack(uOdometry))
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
    if mcm.get_walk_ismoving()==1 then
      poseFilter.addNoise()
    end
  end
  -- If the ball is detected
	ball = detected.ball
  if ball then
     -- print(string.format('ball BEFORE filter: %.1f, %.1f\n',
     --   ball.v[1], ball.v[2]))
    ballFilter.observation_xy(ball.v[1], ball.v[2], ball.dr, ball.da, ball.t)
     -- print(string.format('ball AFTER filter: %.1f, %.1f\n',
     --   wcm.get_ball_x(), wcm.get_ball_y()))
  end
  -- If the goal is detected
  -- If robot is static then do not use posts for localization
	goal = detected.posts
  if goal and mcm.get_walk_ismoving()==1 then
    if goal[1].type == 3 then
      goal_type_to_filter[goal[1].type]({goal[1].v, goal[2].v})
    else
      goal_type_to_filter[goal[1].type]({goal[1].v, vector.zeros(4)})
    end
  end
  -- If the obstacle is detected
  obstacle = detected.obstacles
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
  -- Processing count
  count = 0
end

function libWorld.update(uOdom, detection)
  local t = unix.time()
  -- Run the updates
  if IS_WEBOTS then
    -- TODO: Add webots specific functions
    -- For SJ: This includes any GPS usage
  end

  update_odometry(uOdom)
  update_vision(detection)
  
  -- Update pose in wcm
  wcm.set_robot_pose(vector.pose{poseFilter.get_pose()})
  
  -- Increment the process count
  count = count + 1
end

function libWorld.send()
  local to_send = {}
  to_send.info = ''
  -- Robot info
  to_send.pose = vector.new(wcm.get_robot_pose())
  to_send.info = to_send.info..string.format(
    'Pose: %.2f %.2f (%.1f)\n', to_send.pose[1], to_send.pose[2], to_send.pose[3]*RAD_TO_DEG)
    
    -- print('libWorld, odom:', unpack(wcm.get_robot_odometry()))
  
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
  
  if obstacle then
    local obs = {}
    for i=1, #obstacle.v do
      obs[i] = obstacle.v
      to_send.info = to_send.info..string.format(
        'Obstacle: %.1f %.1f\n', unpack(obstacle.v[i]) )
    end
    to_send.obstacle = obs
  end
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
