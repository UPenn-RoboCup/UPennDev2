local libWorld = {}

--SJ: a dummy world file
--we can add slam, odometry, and so on later

-- TODO: Add Attack bearing
-- TODO: Add Webots ground truth knowledge
local Body   = require'Body'
local vector = require'vector'
local util = require'util'
local odomScale = Config.world.odomScale 
local use_imu_yaw = Config.world.use_imu_yaw


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
--TODO: need posefilter for slam + odometry
--  poseFilter.odometry(unpack(uOdometry))
end

local function update_vision()  end


function libWorld.entry()
  t_entry = unix.time()
  -- Initialize the pose filter  
--TODO: init pose  
--  poseFilter.initialize()

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
  print"SSS"
  if IS_WEBOTS and Config.use_gps_pose then
    wcm.set_robot_pose(wcm.get_robot_pose_gps())

    print("gps pose:",unpack(wcm.get_robot_pose()))

  else
    update_odometry(uOdom)
  end
  -- Increment the process count
  count = count + 1



end


function libWorld.send()
  local to_send = {}
  to_send.info = ''
  -- Robot info
  to_send.pose = vector.new(wcm.get_robot_pose())
  to_send.time = Body.get_time()
  to_send.info = to_send.info..string.format(
    'Pose: %.2f %.2f (%.1f)\n', to_send.pose[1], to_send.pose[2], to_send.pose[3]*RAD_TO_DEG)
  to_send.time = Body.get_time()
  return to_send
end

function libWorld.exit()
end

function libWorld.get_pose()
--TODO

  return vector.pose({0,0,0})
  --return vector.pose{poseFilter.get_pose()}
end

libWorld.update_odometry = update_odometry
libWorld.update_vision = update_vision

return libWorld
