-- A RoboCup World
local libWorld = {}

-- TODO: Add Attack bearing
-- TODO: Add Webots ground truth knowledge
local vector = require'vector'
local ballFilter = require'ballFilter'
local poseFilter = require'poseFilter'
local odomScale = Config.world.odomScale
local use_imu_yaw = Config.world.use_imu_yaw
local RESAMPLE_PERIOD = Config.world.resample_period
local RESAMPLE_COUNT = Config.world.resample_count

-- Timestamps
local t_entry

-- Cycle count
local count

-- Initial odometry
local uOdometry0 = vector.zeros(3)
-- Save the resampling times
local t_resample = 0

local function update_odometry(uOdometry)
  -- Scale the odometry
  uOdometry[1] = odomScale[1] * uOdometry[1]
  uOdometry[2] = odomScale[2] * uOdometry[2]
  uOdometry[3] = odomScale[3] * uOdometry[3] * DEG_TO_RAD
  -- Next, grab the gyro yaw
  if use_imu_yaw then
    local yaw = Body.get_gyro(3)
    yaw0 = yaw
    uOdometry[3] = yaw - yaw0
  end
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
    poseFilter.addNoise()
  end
  -- If the ball is detected
	local ball = detected.ball
  if ball then
     print(string.format('ball BEFORE filter: %.1f, %.1f\n',
       ball.v[1], ball.v[2]))
    ballFilter.observation_xy(ball.v[1], ball.v[2], ball.dr, ball.da, ball.t)
     print(string.format('ball AFTER filter: %.1f, %.1f\n',
       wcm.get_ball_x(), wcm.get_ball_y()))
  end
  -- If the goal is detected
	local goal = detected.posts
  if goal then
    if goal[1].type == 3 then
      goal_type_to_filter[goal[1].type]({goal[1].v, goal[2].v})
    else
      goal_type_to_filter[goal[1].type]({goal[1].v, vector.zeros(4)})
    end
  end
end

function libWorld.entry()
  t_entry = unix.time()
  -- Initialize the pose filter
  poseFilter.initialize_unified()
  -- Save this resampling time
  t_resample = t_entry
  -- TODO: Set the initial odometry
  -- Processing count
  count = 0
end

function libWorld.update(uOdom, detection)
  local t = unix.time()
  -- Grab the pose before updating
  local pose0 = vector.pose{poseFilter.get_pose()}
  -- Run the updates
  if IS_WEBOTS then
    -- TODO: Add webots specific functions
    -- For SJ: This includes any GPS usage
  end
  update_odometry(uOdom)
  update_vision(detection)

  -- Increment the process count
  count = count + 1
end

function libWorld.exit()
end

function libWorld.get_pose()
  return vector.pose{poseFilter.get_pose()}
end

libWorld.update_odometry = update_odometry
libWorld.update_vision = update_vision

return libWorld
