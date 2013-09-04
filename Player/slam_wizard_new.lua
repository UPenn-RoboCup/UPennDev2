---------------------------------
-- Behavior manager for Team THOR
-- (c) Stephen McGill, 2013
---------------------------------

-- Set the path for the libraries
dofile'../include.lua'
local Config = require'Config'
local new_body = require'Body'
local ConfigPenn = require'ConfigPenn'
-- TODO: make sure Body works with gazebo, too
--local Body = require(ConfigPenn.Body)
require 'unix'

-- Set the Debugging mode
local debugging = true
local Benchmark = false --true

-- Set the real-robot mode
local realFlag = 1
-- Replay log?
logFile = false --false
IS_WEBOTS = true

---------------------------------
-- Libraries
local udp = require 'udp' -- Needs to be before simple_ipc for some reason
local simple_ipc = require'simple_ipc'
local carray = require'carray'
local libLaser = require'libLaser'
local libSlam = require'libSlam'
local mp = require'msgpack'
--local tutil = require'tutil' 
local msg = require'msgpack'
--local jcm = require'jcm'
local jpeg = require'jpeg'
--local scm = require'scm'
local util = require'util'
local udp = require'udp'
local udp_port = Config.net.omap -- TODO: add hmap later
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
--TODO: Data structures


---------------------------------
-- IPC channels
local lidar_channel_0 = simple_ipc.new_subscriber('head_lidar')
local lidar_channel_1 = simple_ipc.new_subscriber('chest_lidar')
---------------------------------

---------------------------------
-- Initialize lidars
-- TODO: add function setup_lidar() for general initialization
-- Arguments: location, minRange, maxRange, minHeight, maxHeight
-- TODO: add min/max angles as well?
-- l0: chest  l1: head
local l0minIdx = 1
local l0maxIdx = 1081
local l1minIdx = 0
local l1maxIdx = 0
 
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

local lidar0 = 
libLaser.new_lidar('head', minRange, maxRange, l0minHeight, l0maxHeight, l0minIdx, l0maxIdx)
local lidar1 = 
libLaser.new_lidar('chest', minRange, maxRange, 0,0, l1minIdx, l1maxIdx)
---------------------------------

---------------------------------
-- Process the head lidar
lidar0_count = 0
lidar0_interval = 3 -- process every 2 frames

lidar1_count = 0
lidar1_interval = 3 -- process every 2 frames

time_match = 0;
time_total = 0;
proc_count = 0;

pre_pose = {0,0,0}


--libSlam.SLAM.xOdom = libSlam.SLAM.xOdom + dx
lidar_channel_0.callback = function()
	
	-- Grab the data	
	local ts, has_more = lidar_channel_0:receive()
	local lidar_ts = tonumber(ts);
	if not has_more then
		local debug_msg = string.format('LIDAR (%.2f) | ', unix.time() )
		print(debug_msg.."Bad 0 ts", type(ts))
		return
	end
	local ranges_str, has_more = lidar_channel_0:receive()
	
	if realFlag==1 then
		if not has_more then
			local debug_msg = string.format('LIDAR (%.2f) | ', unix.time() )
			print(debug_msg.."Bad 0 ranges_str", type(ranges_str))
			return
		end
		local metadata, has_more = lidar_channel_0:receive()
		local meta = mp.unpack( metadata )
		cur_pose = meta[1]
		neck = meta[2]
	else
		neck = Body.get_neck_position()
		cur_pose = scm:get_pose_odom()
	end
	
		
	-- Don't slam all the time
	lidar0_count = lidar0_count + 1;
	if lidar0_count%lidar0_interval~=0 then
		return
	end
	
	------------------
	-- Benchmark
	local t0 = unix.time()
	------------------
	
	---------------------------------
	-- Reduce angle to [-pi, pi)
	---[[
	cur_pose[3] = cur_pose[3] % (2*math.pi)
	if (cur_pose[3]  >= math.pi) then
		cur_pose[3]  = cur_pose[3]  - 2*math.pi
	end
	--]]
	---------------------------------
	
	---[[
	---------------------------------
	-- Find out how much we have moved since last slamming
	--pre_pose = pre_pose or cur_pose
	local dx = cur_pose[1] - pre_pose[1]
	local dy = cur_pose[2] - pre_pose[2]
	local da =  cur_pose[3] - pre_pose[3] --util.mod_angle( cur_pose[3] - pre_pose[3] )
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
	--]]
	------------------------------------------------------
	-- Do not slam when the head is moving
	local head_roll,head_pitch,head_yaw = 0,neck[1],neck[2]
	if realFlag ==1 and (math.abs(head_pitch)>math.pi/180 or math.abs(head_yaw)>math.pi/180) then
		--print('bad head!!!!')
		scm:set_pose_slam({libSlam.SLAM.xOdom,libSlam.SLAM.yOdom,libSlam.SLAM.yawOdom})
		return
	end
	------------------------------------------------------
	
	------------------
	-- Copy received data to the lidar object
	-- Transform the points into the right frame
	--tutil.memcpy( lidar0.ranges:storage():pointer(), ranges_str )
	-- TODO: I don't think this will work well
	tutil.memcpy_ranges_fov( lidar0.ranges, ranges_str, l0minIdx, l0maxIdx )
	lidar0:transform( head_roll, head_pitch, head_yaw )
	------------------
		
	-- Scan match
	local t0_processL0 = unix.time()
	libSlam.processL0( lidar0.points_xyz )
	local t1_processL0 = unix.time()
	--print( string.format('processL0 took: \t%.2f ms', (t1_processL0-t0_processL0)*1000) )
	------------------

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
	proc_count = proc_count + 1;
  
end



------------------------------------------------------
-- Chest lidar callback
------------------------------------------------------
lidar_channel_1.callback = function()
	-- Grab the data
	local meta, has_more = lidar_channel_1:receive()
	local metadata = mp.unpack(meta)
	local ranges_str = new_body.get_chest_lidar()
	
	-- TODO: chest_yaw? scanlines?
	if IS_WEBOTS then
		cur_pose = wcm.get_robot_pose() -- Ground truth pose
	elseif USE_SLAM_ODOM then
	  --cur_pose = scm/blah:get_pose()
	end

  --[[
	local ts, has_more = lidar_channel_1:receive()
	local lidar_ts = tonumber(ts);
	if not has_more then
		local debug_msg = string.format('LIDAR (%.2f) | ', unix.time() )
		print(debug_msg.."Bad 1 ts", type(ts))
		return
	end

	local ranges_str, has_more = lidar_channel_1:receive()	
	if not logFile and realFlag==1 then
		if not has_more then
			local debug_msg = string.format('LIDAR (%.2f) | ', unix.time() )
			print(debug_msg.."Bad 1 ranges_str", type(ranges_str))
			return
		end
		local metadata, has_more = lidar_channel_1:receive()
		local meta = mp.unpack( metadata )
		cur_pose = meta[1]
		chest_yaw = meta[2]
	else
		-- Gazebo
		
		chest_yaw = Body.get_lidar_position()[1]
		if realFlag == 0 then
			chest_yaw = -1*(chest_yaw-math.pi)*180/math.pi
		end
		cur_pose = scm:get_pose_odom()
	end
	--]]
	
	
	-- Don't process every time
	lidar1_count = lidar1_count + 1;
	if lidar1_count%lidar1_interval~=0 then
		return
	end

	------------------
	-- Copy received data to the lidar object
	-- Transform the points into the body frame and
	-- prune based on Indices for angles
	-- TODO: I think this may cause memory access issues
	
	--tutil.memcpy_ranges_fov( lidar1.ranges, ranges_str, l1minIdx, l1maxIdx )
	lidar1.ranges = torch.DoubleTensor(1080, 1):zero() -- TODO: stupid testing

	-- Transform the points into the body frame
	------------------
	-- Get the chest lidar current pose
	local chest_roll = 0
	local chest_pitch = 0
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

------------------
-- Perform the loop
local cnt = 0

local slam = {}

function slam.entry()
  -- Set up data structure for omap

  --[[ Poll lidar readings
  local wait_channels = {}
  if head.lidar_ch then
    head.lidar_ch.callback = head_callback
    table.insert( wait_channels, head.lidar_ch )
  end
  if chest.lidar_ch then
    chest.lidar_ch.callback = chest_callback
    table.insert( wait_channels, chest.lidar_ch )
  end
  --]]

  local wait_channels = {lidar_channel_1}

  -- Set up omap messages sender on UDP
  omap_udp_ch = udp.new_sender( udp_target, udp_port )
  channel_polls = simple_ipc.wait_on_channels( wait_channels )
end

function slam.update()
	------------------
	-- Merge maps
	------------------
	libSlam.mergeMap()
	
	
	------------------
	-- Compress and send the SLAM map and pose over UDP
	local jomap = jpeg.compress_gray(
	libSlam.SMAP.data:storage():pointer(),
	libSlam.MAPS.sizex,
	libSlam.MAPS.sizey
	)
	
	local metadata={
		Xmin = libSlam.SMAP.xmin,
		Ymin = libSlam.SMAP.ymin,
		Xmax = libSlam.SMAP.xmax,
		Ymax = libSlam.SMAP.ymax
	}

  -- TODO: compression like in mesh_wizard?
	data = mp.pack({image=jomap,data=metadata})
	local udp_ret, err = omap_udp_ch:send( data, #data )
	print(err or 'Omap data sent!')
	------------------
 
	------------------
	-- Perform the poll (Slam Processing)
	local npoll = channel_polls:poll(channel_timeout)
	--print('poll', npoll)
	local t = unix.time()
	cnt = cnt+1;
	------------------
	
	------------------
	-- Debugging output
	if t-t_last>t_debug then
		local msg = string.format(
		"\nSlam Manager (%.2f) | %.2f FPS", t, cnt/t_debug
		);
		local pose_debug = string.format(
		'My pose is:\t %.2f,%.2f, %.2f',
		libSlam.SLAM.xOdom, libSlam.SLAM.yOdom, libSlam.SLAM.yawOdom
		)
		
		--print(msg)
		--print(pose_debug)
		t_last = t;
		cnt = 0;
	end
	-----------------
	
end

function slam.exit()
end

-- Main loop
slam.entry()
while true do slam.update() end
slam.exit()

return slam
