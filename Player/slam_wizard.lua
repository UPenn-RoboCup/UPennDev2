---------------------------------
-- SLAM wizard for Team THOR
-- (c) Stephen McGill, 2013
---------------------------------
dofile'../include.lua'
local simple_ipc = require'simple_ipc'
local mp = require'msgpack'
local vector = require'vector'
local util = require'util'
local cutil = require'cutil'
local Body = require'Body'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local libMap = require'libMap'

-- Open the map to localize against
local map = libMap.open_map'map.ppm'
map.pose = vector.pose{-1.32058, -0.216679, 1.5708}

-- Render so that I can see it :)
if DO_EXPORT then
	local c_map = map:render'png'
	local f_map = io.open('cur_map.png','w')
	f_map:write(c_map)
	f_map:close()
	libMap.export(map.omap,'omap.raw','byte')
end

local function localize(ch)
	--local pose_gps = vector.pose( wcm.get_robot_gps() )
	local matched_pose, hits = map:localize( ch.points, {} )
	map.pose = matched_pose
	wcm.set_robot_pose( matched_pose )
	--print('\nMatched',matched_pose)
	--print('GPS',pose_gps)
end

-- Listen for lidars
local wait_channels = {}
local lidar_ch = simple_ipc.new_subscriber'lidar'
lidar_ch.n   = 769 -- Webots: 721
lidar_ch.fov = 270 -- Webots: 180
lidar_ch.res = 360 / 1024
lidar_ch.angles = torch.range(0,lidar_ch.fov,lidar_ch.res)*DEG_TO_RAD
assert(lidar_ch.n==lidar_ch.angles:size(1),"Bad lidar resolution")
lidar_ch.raw     = torch.FloatTensor(lidar_ch.n)
lidar_ch.ranges  = torch.Tensor(lidar_ch.n)
lidar_ch.cosines = torch.cos(lidar_ch.angles)
lidar_ch.sines   = torch.sin(lidar_ch.angles)
lidar_ch.points  = torch.Tensor(lidar_ch.n,2)
lidar_ch.callback = function(sh)
	local ch = wait_channels.lut[sh]
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
	assert(ch.raw:size(1)==meta.n,"TODO: Update range sizes")
	cutil.string2storage(ranges,ch.raw:storage())
	-- TODO: Filter... r and theta
	local ir = 0
	ch.ranges:apply(function(x)
			ir = ir+1
			-- Only use 180 degrees of data, to avoid self collision
			if ir<129 or ir>641 then return 0 end
			return x
		end)
	-- For now, just copy
	ch.ranges:copy(ch.raw)
	-- Put into x y space from r/theta
	local pts_x = ch.points:select(2,1)
	local pts_y = ch.points:select(2,2)
	torch.cmul(pts_x,ch.sines,ch.ranges)
	torch.cmul(pts_y,ch.cosines,ch.ranges)
	-- Link length
	pts_x:add(.3)
	-- Localize based on this channel
	localize(ch)

	-- Save the xy lidar points
	if SAVE_POINTS then
		local f = io.open('xy.raw', 'w')
		local ptr, n_el = ch.points:storage():pointer(), #ch.points:storage()
		local arr = carray.double(ptr, n_el)
		f:write( tostring(arr) )
		f:close()
	end

end

-- Make the poller
table.insert(wait_channels, lidar_ch)
local channel_timeout = 0.5 * 1e3
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
while true do channel_poll:poll(channel_timeout) end
