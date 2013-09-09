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
local udp = require 'udp' -- Needs to be before simple_ipc for some reason
local simple_ipc = require'simple_ipc'
local carray = require'carray'
local libLaser = require'libLaser'
local libSlam = require'libSlam'
local mp = require'msgpack'
local msg = require'msgpack'
--local jcm = require'jcm'
local jpeg = require'jpeg'
local zlib = require'zlib'
local util = require'util'
local udp = require'udp'
-- TODO: add hmap later or just use merged map?
local udp_port = Config.net.omap
local udp_target = Config.net.operator.wireless
jpeg.set_quality( 90 ) -- 90? other ?
---------------------------------

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
if realFlag == 1 then
	l1minIdx = 541 --541
	l1maxIdx = 781 --781
else
	l1minIdx = 201 --541
	l1maxIdx = 881 --781
end

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

time_match = 0;
time_total = 0;

pre_pose = {0,0,0}


-- Setup metadata and tensors for a lidar readings
local function setup_lidar( name )
  local tbl = {}
  -- Save the meta data for easy sending
  tbl.meta = {}
  tbl.meta.name = name

	tbl.meta.rpy = wcm.get_robot_rpy()
	-- Default yaw is 1.57
	tbl.meta.rpy[3] = tbl.meta.rpy[3] - math.pi/2
	tbl.meta.gyro = wcm.get_robot_gyro()
	print('GYRO:', unpack(tbl.meta.gyro))

  -- Actuator endpoints
  -- In radians, specifices the actuator scanline angle endpoints
  -- The third number is the scanline density (scanlines/radian)
  tbl.meta.scanlines = vcm['get_'..name..'_lidar_scanlines']()

  -- Field of view endpoints of the lidar ranges
  -- -135 to 135 degrees for Hokuyo
  -- This is in RADIANS
  tbl.meta.fov = vcm['get_'..name..'_lidar_fov']()


  -- Depths when compressing TODO: may not needed
  tbl.meta.depths = vcm['get_'..name..'_lidar_depths']()
  -- Type of compression
  tbl.meta.c = 'jpeg'
  -- Timestamp
  tbl.meta.t = Body.get_time()

  -- Find the resolutions
  local scan_resolution = tbl.meta.scanlines[3]
    * math.abs(tbl.meta.scanlines[2]-tbl.meta.scanlines[1])
  scan_resolution = math.ceil(scan_resolution)
  local reading_per_radian = 1 / (.25*math.pi/180)
  local fov_resolution = reading_per_radian
    * math.abs(tbl.meta.fov[2]-tbl.meta.fov[1])
  fov_resolution = math.ceil(fov_resolution)
  -- Resolution
  tbl.meta.resolution = {scan_resolution,fov_resolution}

  -- Conver RADIANS to INDEX
  tbl.meta.fov_idx = {0, 0}
  tbl.meta.fov_idx[1] = reading_per_radian
    * ( tbl.meta.fov[1] - (-135)*deg2rad ) + 1 
  tbl.meta.fov_idx[1] = math.floor(tbl.meta.fov_idx[1])
  tbl.meta.fov_idx[2] = math.min( tbl.meta.fov_idx[1] + fov_resolution - 1, 1081)
  --print('fov_idx', unpack(tbl.meta.fov_idx))
  --TODO: make sure if the order needs to be flipped

  -- Setup lidar object
  -- Arguments: location, minRange, maxRange
  -- minHeight, maxHeight, minIdx, maxIdx
  if name == 'head' then
    lidar0 = 
    libLaser.new_lidar(
      'head', 
      minRange, maxRange, 
      l0minHeight, l0maxHeight, 
      tbl.meta.fov_idx[1], tbl.meta.fov_idx[2] )
  else
    lidar1 = 
    libLaser.new_lidar(
      'chest', 
      minRange, maxRange, 
      0,0, 
      tbl.meta.fov_idx[1], tbl.meta.fov_idx[2] )
  end

  -- TODO: We don't need to keep all ranges...?
  --tbl.all_ranges = torch.FloatTensor( scan_resolution, fov_resolution ):zero()
  tbl.range = torch.FloatTensor(1, fov_resolution):zero()
  -- TODO: Save the exact actuator angles?
  tbl.scan_angles  = torch.DoubleTensor( scan_resolution ):zero()

  -- Subscribe to a lidar channel
  tbl.lidar_ch  = simple_ipc.new_subscriber(name..'_lidar')

  -- Find the offset for copying lidar readings into the torch object
  tbl.offset_idx = tbl.meta.fov_idx[1] - 1
  print('LIDAR OFFSET', name, tbl.offset_idx)
  -- For streaming
  tbl.needs_update = true

  return tbl
end

------------------------------
-- Data copying helpers
-- Convert a pan angle_to_scanline to a column of the chest mesh image
local function angle_to_scanline( meta, rad )
  local start = meta.scanlines[1]
  local stop  = meta.scanlines[2]
  local res   = meta.resolution[1]
  local ratio = (rad-start)/(stop-start)
  -- Round
  --local scanline = math.floor(ratio*res+.5)
  local scanline = math.ceil(ratio*res)
  -- Return a bounded value
  return math.max( math.min(scanline, res), 1 )
end
------------------------------


local function head_callback()
	-- Don't slam all the time
	lidar0_count = lidar0_count + 1;
	if lidar0_count % lidar0_interval~=0 then
		return
	end

	-- TODO: head_pitch
	if IS_WEBOTS then
		cur_pose = wcm.get_robot_pose() -- Ground truth pose
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
	
	-- Grab the data	
    local meta, has_more = head.lidar_ch:receive()
    local metadata = mp.unpack(meta)
    -- Get raw data from shared memory
    local ranges = Body.get_head_lidar()
    -- TODO: You don't need scanlines...
    -- Insert into the correct scanlin
    local angle = metadata.angle
    local scanline = angle_to_scanline( head.meta, angle )
    -- Only if a valid column is returned
    if scanline then
      -- Copy lidar readings to the torch object for fast modification
      ranges:tensor( 
        --head.all_ranges:select(1, scanline),
        head.range:select(1,1),
        head.range:size(2),
        head.offset_idx
      )
      -- Save the pan angle
      head.scan_angles[scanline] = angle
      head.needs_update = true
      head.meta.t = metadata.t

      -- Converstion of ranges for use in libLaser
      -- TODO: copy may be slow
      --local single = torch.FloatTensor(1, head.all_ranges:size(2)):zero()
      --single:copy(head.all_ranges:select(1, scanline))
      lidar0.ranges:copy(head.range:transpose(1,2))
    end
	
	local head_roll, head_yaw = 0, 0 
	--FIXME: when scan_angle=0, the head_pitch is not zero
	local head_pitch = head.scan_angles[scanline] + 5*deg2rad
	--print(string.format('\nHEAD PITCH:\t%.2f', head_pitch*180/math.pi))
	lidar0:transform( head_roll, head_pitch, head_yaw )
	------------------
		
	-- Scan match
	local t0_processL0 = unix.time()
	-- TODO: use head.meta.t for slam
	-- TODO: use RPY from webots
	-- TODO: Just add the gyro values to the lidar metadata
	libSlam.processIMU( head.meta.rpy, head.meta.gyro[3], head.meta.t )
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
  
	time_total = time_match + (t1-t0);
  
end



------------------------------------------------------
-- Chest lidar callback
------------------------------------------------------
local function chest_callback()
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
	-- Grab the data
	local meta, has_more = chest.lidar_ch:receive()
	local metadata = mp.unpack(meta)
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
	local chest_roll = 0
	local chest_pitch = 0
	local chest_yaw = chest.scan_angles[scanline]
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
local t = unix.time()
local t_last = t
local t_debug = 1 -- Print debug output every second
------------------

------------------
-- No debugging messages
-- Only the callback will be made
if not debugging then
	channel_polls:start()
end
------------------


local slam = {}

function slam.entry()
  -- Specify desired FOV
  vcm.set_head_lidar_fov({-135*deg2rad, 135*deg2rad})
  vcm.set_chest_lidar_fov({-60*deg2rad, 60*deg2rad})

  -- Set up data structure for each lidar
  chest = setup_lidar('chest')
  head = setup_lidar('head')

  -- Poll lidar readings
  local wait_channels = {}
  if head.lidar_ch then
    head.lidar_ch.callback = head_callback
    table.insert( wait_channels, head.lidar_ch )
  end
  if chest.lidar_ch then
    chest.lidar_ch.callback = chest_callback
    --table.insert( wait_channels, chest.lidar_ch )
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
  if err then print(err) end
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
