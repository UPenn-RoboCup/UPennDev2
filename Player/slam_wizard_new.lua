---------------------------------
-- Behavior manager for Team THOR
-- (c) Stephen McGill, 2013
---------------------------------

-- Set the path for the libraries
dofile'../include.lua'
local Config = require'Config'
-- TODO: make sure Body works with gazebo, too
local Body = require'Body'
require 'unix'

-- Set the Debugging mode
local debugging = true
local Benchmark = false --true

-- Set the real-robot mode
local realFlag = 1
local deg2rad = math.pi/180
IS_WEBOTS = true

---------------------------------
-- Libraries
local udp = require 'udp'
-- udp must be before simple_ipc for some reason?
local simple_ipc = require'simple_ipc'
local carray = require'carray'
local libLaser = require'libLaser'
local libSlam = require'libSlam'
local mp = require'msgpack'
local jpeg = require'jpeg'
local zlib = require'zlib'
local util = require'util'
local vector = require'vector'
local udp  = require'udp'
-- TODO: add hmap later or just use merged map?
local udp_port = Config.net.omap
local udp_target = Config.net.operator.wireless
jpeg.set_quality( 90 ) -- 90? other ?
---------------------------------
local head_lidar_ch = simple_ipc.new_subscriber'head_lidar'
--local chest_lidar_ch = simple_ipc.new_subscriber'chest_lidar'

---------------------------------
-- Shared Memory
require'wcm'
---------------------------------


-- Output Channels
local omap_udp_ch
-- Input Channels
local channel_polls
local channel_timeout = 100 -- ms

-- Lidar objects
-- TODO: renames as head and chest
local lidar0 -- head
local lidar1 -- chest

---------------------------------
-- Filter Parameters
-- TODO: the following should be put in vcm 
local l0minFOV = -135*Body.DEG_TO_RAD
local l0maxFOV =  135*Body.DEG_TO_RAD
local l1minFOV = -45*Body.DEG_TO_RAD
local l1maxFOV =  45*Body.DEG_TO_RAD
local l0minHeight = -0.6 -- meters
local l0maxHeight = 1.2
-- We don't need height limits for chest lidar
local minRange = 0.15 -- meters
local maxRange = 28

---------------------------------
-- Lidar processing params
lidar0_count = 0
lidar0_interval = 3 -- process every # frames

lidar1_count = 0
lidar1_interval = 3 -- process every # frames

time_match = 0
time_total = 0

pre_pose = {0,0,0}

---------------------------------
-- Callbacks for receiving lidar readings
local function head_callback()
  -- Grab the data  
  local meta, has_more = head_lidar_ch:receive()
  local metadata = mp.unpack(meta)
  -- Grab the pitch angle (if off center, then maybe just escape)
  local angle = metadata.angle
  --head.meta.t = metadata.t

  -- Don't slam all the time
  lidar0_count = lidar0_count + 1;
  if lidar0_count%lidar0_interval~=0 then return end

  -- TODO: head_pitch
  if IS_WEBOTS then
    -- Ground truth pose
    cur_pose = wcm.get_robot_pose()
  elseif USE_SLAM_ODOM then
    --cur_pose = scm/blah:get_pose()
  end
    
  ---------------------------------
  -- Reduce angle to [-pi, pi)
  --[[ TODO: util.mod_angle()
  cur_pose[3] = cur_pose[3] % (2*math.pi)
  if (cur_pose[3]  >= math.pi) then
    cur_pose[3]  = cur_pose[3]  - 2*math.pi
  end
  --]]
  ---------------------------------
  
  --[[
  ---------------------------------
  -- Find out how much we have moved since last slamming
  --pre_pose = pre_pose or cur_pose
  local dx = cur_pose[1] - pre_pose[1]
  local dy = cur_pose[2] - pre_pose[2]
  local da =  cur_pose[3] - pre_pose[3]
  -- Save the previous pose
  pre_pose = cur_pose
  ---------------------------------
  
  ---------------------------------
  -- If we did not move much, or jumped, then do not update the odom
  if math.abs(cur_pose[3])>1e-5 and math.abs(dx) < 0.5 and math.abs(dy)<0.5 and math.abs(da) < 0.2 then
  --if odomFlag then
    libSlam.SLAM.xOdom = libSlam.SLAM.xOdom + dx
    libSlam.SLAM.yOdom = libSlam.SLAM.yOdom + dy
    libSlam.SLAM.yawOdom = libSlam.SLAM.yawOdom + da -- util.mod_angle( libSlam.SLAM.yawOdom + da )
    --print('good odom!!!!',dx,dy,da)
  else
    --print('bad odom!!!!',dx,dy,da)
    --print('SLAM_before',libSlam.SLAM.xOdom,libSlam.SLAM.yOdom,libSlam.SLAM.yawOdom)
  end
  ---------------------------------
  
  ------------------------------------------------------
  -- Do not slam when the head is moving
  local head_roll,head_pitch,head_yaw = 0,neck[1],neck[2]
  if realFlag ==1 and (math.abs(head_pitch)>math.pi/180 or math.abs(head_yaw)>math.pi/180) then
    --print('bad head!!!!')
    wcm:set_pose_slam({libSlam.SLAM.xOdom,libSlam.SLAM.yOdom,libSlam.SLAM.yawOdom})
    return
  end
  ------------------------------------------------------
  --]]
  
  ----------------
  -- Benchmark
  t0 = unix.time()
  ----------------
  
  --head.needs_update = true

  -- Get raw data from shared memory
  -- TODO: May try to put into the lidar message itself
  -- which is useful for a separate computer to perform slam
  local ranges = Body.get_head_lidar()
  print('lidar sizes',#ranges,lidar0.ranges:size(1))
  -- Copy of ranges for use in libLaser
  ranges:tensor( lidar0.ranges )
  
  -- TODO: Add the yaw from the head
  local head_roll, head_pitch, head_yaw = 0, metadata.angle, 0 
  --print(string.format('\nHEAD PITCH:\t%.2f', head_pitch*180/math.pi))
  lidar0:transform( head_roll, head_pitch, head_yaw )
  ------------------
    
  -- Scan match
  local t0_processL0 = unix.time()
  -- TODO: use head.meta.t for slam
  -- TODO: use RPY from webots
  -- TODO: Just add the gyro values to the lidar metadata
  print('RPY/Gyro',vector.new(metadata.rpy), metadata.gyro[3])
  libSlam.processIMU( metadata.rpy, metadata.gyro[3], metadata.t )
  libSlam.processL0( lidar0.points_xyz )
  local t1_processL0 = unix.time()
  --print( string.format('processL0 took: \t%.2f ms', (t1_processL0-t0_processL0)*1000) )
  ------------------
  
  --wcm:set_pose_slam({libSlam.SLAM.xOdom,libSlam.SLAM.yOdom,libSlam.SLAM.yawOdom})

  ------------------
  --[[ For real robot
  time_match = time_match + (t1_processL0-t0_processL0);
  scm:set_pose_slam_time( t1_processL0 )
  scm:set_pose_slam({libSlam.SLAM.xOdom,libSlam.SLAM.yOdom,libSlam.SLAM.yawOdom})
  if logFile or not realFlag then
    scm:set_pose({libSlam.SLAM.xOdom,libSlam.SLAM.yOdom,libSlam.SLAM.yawOdom})
  end
  --print('SLAM_after ',libSlam.SLAM.xOdom,libSlam.SLAM.yOdom,libSlam.SLAM.yawOdom)
  --]]
  ------------------

  local t1 = unix.time()
  if Benchmark then
    print(string.format('SlamL0 took: \t\t%.2f ms',(t1-t0)*1000))
  end
  time_total = time_match + (t1-t0)
end



------------------------------------------------------
-- Chest lidar callback
------------------------------------------------------
local function chest_callback()

  -- Grab the data
  local meta, has_more = chest_lidar_ch:receive()
  local metadata = mp.unpack(meta)

if true then return end

  -- Don't process every time
  lidar1_count = lidar1_count + 1;
  if lidar1_count%lidar1_interval~=0 then
    return
  end
    
  if IS_WEBOTS then
    cur_pose = wcm.get_robot_pose() -- Ground truth pose
  elseif USE_SLAM_ODOM then
    --cur_pose = scm/blah:get_pose()
  end

  ------------------
  -- Copy received data to the lidar object
  -- Transform the points into the body frame and
  -- prune based on Indices for angles
  
    -- Get raw data from shared memory
  local ranges = Body.get_chest_lidar()
  
    -- Insert into the correct column
    local angle = metadata.angle
    local scanline = angle_to_scanline( chest.meta, angle )
    -- Only if a valid column is returned
    if scanline then
      -- Copy lidar readings to the torch object for fast modification
      ranges:tensor(
        --chest.all_ranges:select(1, scanline),
        chest.range:select(1,1),
        chest.range:size(2),
        chest.offset_idx 
    )
      -- Save the pan angle
      chest.scan_angles[scanline] = angle
      -- We've been updated
      chest.needs_update = true
      chest.meta.t     = metadata.t
      
      -- Converstion of ranges for use in libLaser
      -- TODO: copy may be slow
      --local single = torch.FloatTensor(1, chest.all_ranges:size(2)):zero()
      --single:copy(chest.all_ranges:select(1, scanline))
      lidar1.ranges:copy(chest.range:transpose(1,2))
    end
  
  -- Transform the points into the body frame
  ------------------
  -- Get the chest lidar current pose
  local chest_roll  = 0
  local chest_pitch = 0
  local chest_yaw  = chest.scan_angles[scanline]
  lidar1:transform(chest_roll, chest_pitch, chest_yaw)
  ------------------

  ------------------
  -- Perform Chest Lidar processing
  t0_pL1 = unix.time()
  libSlam.processL1(lidar1.points_xyz)
  t1_pL1 = unix.time()
  
  if Benchmark then
    print( string.format('processL1 took: \t\t%.2f ms', (t1_pL1-t0_pL1)*1000) )
    print('---------------------\n\n')
  end
  ------------------
  
  -- TODO: Use odometry for lidar 0, too.
  pre_pose1 = pre_pose1 or cur_pose
  
end
------------------------------------------------------


------------------
-- Start the timing
local t = Body.get_time()
local t_last = t
local t_debug = 1 -- Print debug output every second
------------------

------------------
-- No debugging messages
-- Only the callback will be made
if not debugging then channel_polls:start() end
------------------


local slam = {}

function slam.entry()

  -- Poll lidar readings
  local wait_channels = {}
  if head_lidar_ch then
    head_lidar_ch.callback = head_callback
    table.insert( wait_channels, head_lidar_ch )
    lidar0 = libLaser.new_lidar(
      'head', 
      minRange, maxRange, 
      l0minHeight, l0maxHeight, 
      l0minFOV, l0maxFOV )
  end
  if chest_lidar_ch then
    chest_lidar_ch.callback = chest_callback
    table.insert( wait_channels, chest_lidar_ch )
    lidar1 = libLaser.new_lidar(
      'chest', 
      minRange, maxRange, 
      l1minHeight, l1maxHeight, 
      l1minFOV, l1maxFOV )
  end

  -- Set up omap messages sender on UDP
  omap_udp_ch = udp.new_sender( udp_target, udp_port )
  channel_polls = simple_ipc.wait_on_channels( wait_channels )
end

local cnt = 0

function slam.update()
  ------------------
  -- Merge maps
  ------------------
  libSlam.mergeMap()
  
  ------------------
  -- Compress and send the SLAM map and pose over UDP
  local c_map
  local meta = {}
  --TODO: get from vcm
  meta.c = 'jpeg'
  --meta.c = 'zlib'
  if meta.c == 'zlib' then
    c_map = zlib.compress(
      libSlam.SMAP.data:storage():pointer(),
      libSlam.SMAP.data:nElement()
    )
  elseif meta.c == 'jpeg' then  
    c_map = jpeg.compress_gray(
      libSlam.SMAP.data:storage():pointer(),
      libSlam.MAPS.sizex,
      libSlam.MAPS.sizey
    )
  else
    return
  end
  
  local shiftdata={
    Xmin = libSlam.SMAP.xmin,
    Ymin = libSlam.SMAP.ymin,
    Xmax = libSlam.SMAP.xmax,
    Ymax = libSlam.SMAP.ymax
  }

  -- Streaming
  meta.shift = shiftdata
  local meta = mp.pack(meta)
  local ret, err = omap_udp_ch:send( meta..c_map )
  if err then print('OMAP send error:',err,#c_map,#meta) end
  ------------------
 
  ------------------
  -- Perform the poll (Slam Processing)
  local npoll = channel_polls:poll(channel_timeout)
  local t = unix.time()
  cnt = cnt+1
  if cnt % 40 == 0 then
    print(string.format('\nSLAM pose:%6.2f %6.2f %6.2f', 
      libSlam.SLAM.xOdom, libSlam.SLAM.yOdom, libSlam.SLAM.yawOdom))
    print(string.format('Scan Match pose:%6.2f %6.2f %6.2f', 
      libSlam.SLAM.x, libSlam.SLAM.y, libSlam.SLAM.yaw))
  end
  ------------------
end

function slam.exit()
end

-- Main loop
slam.entry()
while true do slam.update() end
slam.exit()

return slam
