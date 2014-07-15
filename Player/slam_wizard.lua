---------------------------------
-- SLAM wizard for Team THOR
-- (c) Stephen McGill, 2013
---------------------------------
dofile'../include.lua'
local simple_ipc = require'simple_ipc'
local wait_channels = {}
local mp = require'msgpack'
local vector = require'vector'
local util = require'util'
local cutil = require'cutil'
local Body = require'Body'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local libMap = require'libMap'
local libDetect = require'libDetect'

-- Flags
local USE_ODOMETRY = true
--local SAVE_POINTS  = true
--local DO_EXPORT    = true

-- Open the map to localize against
local map = libMap.open_map(HOME..'/Data/map.ppm')

-- Render so that I can see it :)
if DO_EXPORT==true then
	local c_map = map:render'png'
	local f_map = io.open('cur_map.png','w')
	f_map:write(c_map)
	f_map:close()
	libMap.export(map.omap,'omap.raw','byte')
end

local function setup_ch( ch, meta )
	ch.n   = meta.n
	ch.res = meta.res
	ch.fov = meta.n*meta.res
	local half_view = ch.fov/2
	ch.angles = torch.range(0,ch.n-1):mul(-1*DEG_TO_RAD*ch.res):add(half_view*DEG_TO_RAD)

	assert(ch.n==ch.angles:size(1),"Bad lidar resolution")
	ch.raw     = torch.FloatTensor(ch.n)
	ch.ranges  = torch.Tensor(ch.n)
	ch.cosines = torch.cos(ch.angles)
	ch.sines   = torch.sin(ch.angles)
	ch.points  = torch.Tensor(ch.n,2):zero()
end

local function localize(ch)
	-- Update odometry
	local odom = wcm.get_robot_odometry()
	if USE_ODOMETRY==true then
		local p_traveled = util.pose_relative(odom,map.odom)
		map.pose = util.pose_global(p_traveled,map.pose)
	end
	map.odom = odom
	-- Match laser scan points
	local matched_pose, hits = map:localize( ch.points, {} )
	--print("MATCH",matched_pose,hits)
	map.pose = matched_pose
	return matched_pose
end

-- Listen for lidars
local lidar_cb = function(sh)
	local ch = wait_channels.lut[sh]
	localt = Body.get_time()
	local meta, ranges
	repeat
		-- Do not block
    local metadata, has_more = ch:receive(true)
		-- If no msg, then process
		if not metadata then break end
		-- Must have a pair with the range data
		assert(has_more,"metadata and not lidar ranges!")
		meta = mp.unpack(metadata)
		ranges, has_more = ch:receive(true)
	until false
	-- Update the points
	if meta.n~=ch.n then
		print('Set up the channel!',ch.n,meta.n)
		setup_ch(ch,meta)
	end
	-- Update the count
	ch.count = ch.count + 1
	-- Place into storage
	cutil.string2storage(ranges,ch.raw:storage())
	-- Copy to the double format
	ch.ranges:copy(ch.raw)
	-- Put into x/y space from r/theta
	local pts_x = ch.points:select(2,1)
	local pts_y = ch.points:select(2,2)
	--
	torch.cmul(pts_x,ch.cosines,ch.ranges)
	torch.cmul(pts_y,ch.sines,ch.ranges)
	-- Link length
	pts_x:add(.3)

	-- SLAM
	if wcm.get_map_enable_slam()==1 then
		if not map.pose then
			local start = wcm.get_robot_initialpose()
			-- Need a good guess for the starting pose of the robot
			map.pose = vector.pose(start)
			-- Store a snapshot of the odometry at this time
			map.odom = wcm.get_robot_odometry()
		end
		-- If slam is enabled
		local pose = localize(ch)
		if map.read_only==false then
			-- Update the map
			map:update(ch.points, t)
		end
		print("SLAM POSE",pose)
		wcm.set_robot_pose(pose)
	end
	
	-- Detect circles
	libDetect.lidar_circles(ch)

end

local lidar_ch = simple_ipc.new_subscriber'lidar0'
lidar_ch.name = 'head'
lidar_ch.count = 0
lidar_ch.callback = lidar_cb

-- Make the poller
table.insert(wait_channels, lidar_ch)
local channel_timeout = 0.5 * 1e3
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
while true do channel_poll:poll(channel_timeout) end
