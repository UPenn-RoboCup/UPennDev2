-- libMap
-- (c) 2014 Stephen McGill
-- Navigate within a map
local libMap = {}
local dijkstra = require'dijkstra'
local vector = require'vector'
local carray = require 'carray'
local cutil = require 'cutil'
local util = require 'util'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local png = require'png'
local jpeg = require'jpeg'
local slam = require'slam'
local DEG_TO_RAD = Body.DEG_TO_RAD
local RAD_TO_DEG = Body.RAD_TO_DEG

local function pose_to_map_index(map,pose)
	pose = vector.pose(pose)
	local map_sz = vector.new{map.cost:size(1),map.cost:size(2)}
	local xi = (pose.x - map.bounds_x[1]) * map.inv_resolution
	local yi = (pose.y - map.bounds_y[1]) * map.inv_resolution
	local i = math.max(math.min(math.ceil(xi),map_sz[1]),1)
	local j = math.max(math.min(math.ceil(yi),map_sz[2]),1)
	return i, j
end

local function map_index_to_pose(map,i,j)
	local x = i * map.resolution + map.bounds_x[1]
	local y = j * map.resolution + map.bounds_y[1]
	return vector.pose{x,y,0}
end

local function index_path_to_pose_path(map,i_path,j_path)
	-- Get the current pose
	local cur_pose = wcm.get_robot_pose()
	-- Should return a lua table, not a torch object
	local pose_path = {}
	--local i_mid, j_mid = map.cost:size(1)/2, map.cost:size(2)/2
	local npath = i_path:size(1)
	-- Go in reverse order: last item is is the next point in the path
	local p_after = nil
	-- Easy pop operation in lua
	for p=npath,1,-1 do
		local pose = map_index_to_pose(map,i_path[p],j_path[p])
		--pose.a = cur_pose.a
		local use_wp = true
		--[[
		-- Pruning
		if p<npath and p>1 then
			local p_before = map_index_to_pose(map,i_path[p-1],j_path[p-1])
			local rp = util.pose_relative(p_before,pose)
			local rp_after = util.pose_relative(p_before,p_after)
			local a1 = util.mod_angle(math.atan2(rp[2],rp[1]))
			local a2 = util.mod_angle(math.atan2(rp_after[2],rp_after[1]))
			-- If the same heading, then don't use
			if util.mod_angle(a1-a2)<5*DEG_TO_RAD then use_wp=false end
		end
		--]]
		if use_wp then
			table.insert( pose_path, pose )
			p_after = pose
		end
	end
	return pose_path
end

local function map_to_cost(map,max)
	max = max or 255
	-- Make the double of the cost map
	local cost = torch.DoubleTensor(map:size(1),map:size(2))
	-- Make 1 free space, and max+1 the max?
	-- TODO: Should be log odds... For now, whatever
	cost:copy(map):mul(-1):add(max+1)
	return cost
end

local function map_to_omap(map,max)
	max = max or 255
	-- Make the double of the cost map
	local omap = map:clone()
	omap:apply(function(x)
			if x>100 then return 0 end
			return max-x
		end)
	return omap
end

local render = function( map, fmt )
	-- Export in grayscale
	local export_map = map.map:t():clone()
	local w, h = export_map:size(2), export_map:size(1)
	if fmt=='jpg' or fmt=='jpeg' then
		return jpeg.compress_gray( export_map:storage():pointer(), w, h )
	elseif fmt=='png' then
		return png.compress( export_map:storage():pointer(), w, h, 1 )
	end
end

-- import a map
libMap.open_map = function( map_filename )
	local map = {}
	local f_img = io.open(map_filename,'r')
	local f_type = f_img:read('*line')
	--print('NetPBM type',f_type)
	-- Assume PGM greyscale binary for now
	assert(f_type=='P5' or f_type=='P6','NetPBM binary file support!')
	local is_comment = true
	local comments = {}
	local resolution = nil
	repeat
		comment = f_img:read('*line')
		is_comment = comment:sub(1,1)=='#'
		if is_comment then
			table.insert(comments,comment)
		else
			-- Is resolution
			resolution = comment:gmatch("%d+")
		end
	until not is_comment
	
	-- Parse the resolution iterator
	local ncolumns = tonumber(resolution())
	local nrows    = tonumber(resolution())
	
	-- Maximum value
	local max = tonumber( f_img:read('*line') )
	map.max = max
	
	-- Parse the comments
	assert(#comments>=2,'Need the comments to provide Map resolution and offset')
	-- First comment is map resolution in meters %S is NOT space, %s is space
	local m_res = comments[1]:gmatch("%S+")
	local header = m_res()
	assert(header=='#resolution','Bad resolution header')
	map.resolution = tonumber(m_res())
	assert(map.resolution,'Bad resolution')
	map.inv_resolution = 1 / map.resolution
	-- Second comment are x and y offsets
	local m_offset = comments[2]:gmatch"%S+"
	local header = m_offset()
	assert(header=='#offset','Bad offset header')
	local x_off = tonumber(m_offset())
	assert(x_off,'Bad X offset')
	local y_off = tonumber(m_offset())
	assert(y_off,'Bad Y offset')
	
	-- Read the actual map image
	local img_str = f_img:read('*all')
	
	-- Close the map image file
	f_img:close()

	-- Make the Byte tensor to container the PGM bytes of the map
	local img_t
	if f_type=='P5' then
		-- Grayscale
		assert(#img_str==ncolumns*nrows,'Bad Greyscale resolution check!')
		img_t = torch.ByteTensor(nrows,ncolumns)
		-- Copy the pgm img string to the tensor
		cutil.string2storage(img_str,img_t:storage())
		img_t = img_t:t():clone()
	elseif f_type=='P6' then
		-- RGB
		assert(#img_str==ncolumns*nrows*3,'Bad RGB resolution check!')
		local rgb_t = torch.ByteTensor(nrows,ncolumns,3)
		cutil.string2storage(img_str,rgb_t:storage())
		-- Just the R channel
		img_t = rgb_t:select(3,1):t():clone()
	else
		error('Unsupported!')
	end

	-- Save the map byte image
	map.map = img_t
	-- Make the double of the cost map
	map.cost = map_to_cost(img_t)
	-- Make the occupancy map
	map.omap = map_to_omap(img_t)
	-- Offset is the coordinate of... something
	map.offset = vector.pose{x_off,y_off,0}
	print(map.offset)
	-- Map boundaries
	map.bounds_x = map.resolution*ncolumns/2*vector.new{-1,1}
	map.bounds_y = map.resolution*nrows/2*vector.new{-1,1}
	--print("Bounds",map.bounds_x,map.bounds_y)
	map.bounds_x = vector.new{-map.offset[1]}
	map.bounds_x[2] = map.bounds_x[1]+map.resolution*ncolumns
	map.bounds_y = vector.new{-map.offset[2]}
	map.bounds_y[2] = map.bounds_y[1]+map.resolution*nrows
	--print("Offset",map.bounds_x,map.bounds_y)
	-- Add functions to work on the map itself
	map.new_goal = libMap.new_goal
	map.new_path = libMap.new_path
	map.localize = libMap.localize
	map.grow     = libMap.grow
	map.render   = render
	--
	return map
end

-- Grow the costs so that the robot will not hit anything
libMap.grow = function( map, length, width )
	assert(map.cost,'You must open a map first!')
	--local r_i = math.ceil( map.inv_resolution*.2 )
	local r_i = math.ceil( map.inv_resolution*.32 )
	local r_j = math.ceil( map.inv_resolution*.32 )
	-- Replace the cost map with the grown map
	map.cost = slam.grow_map(map.cost, r_i, r_j)
end

-- Compute the cost to go to the goal
libMap.new_goal = function( map, goal )
	assert(map.cost,'You must open a map first!')
	local i, j = pose_to_map_index(map,goal)
	map.cost_to_go = dijkstra.matrix( map.cost, i, j )
	map.goal = goal
end

-- Compute a path to the goal
libMap.new_path = function( map, start, filename )
	assert(map.cost_to_go,'You must set a goal first!')
	if map.goal==start then return {} end
	local i, j = pose_to_map_index(map,start)
	local i_path, j_path = dijkstra.path( map.cost_to_go, map.cost, i, j )
	if filename then
		local f = io.open(filename, 'w')
		local ptr, n_el = i_path:storage():pointer(), #i_path:storage()
		local arr = carray.int(ptr, n_el)
		f:write( tostring(arr) )
		ptr, n_el = j_path:storage():pointer(), #j_path:storage()
		arr = carray.int(ptr, n_el)
		f:write( tostring(arr) )
		f:close()
	end
	map.start = start
	print("Map Start:",start)
	print("Map Goal:",map.goal)
	local pose_path = index_path_to_pose_path(map,i_path,j_path)
	return pose_path
end

-- x, y laser_point pairs (Nx2 DoubleTensor)
-- search_amount: {x=[radius in meters],y=[radius in meters],
-- a=[radians for angular search], da=[resolution of angular search]}
libMap.localize = function( map, laser_points, search_amount, prior )
	assert(map.pose,'Must have a valid pose on the map!')
	local pose_guess = map.pose
	--
	local dx = search_amount.x or .25
	local ddx = math.floor(dx*map.inv_resolution)* map.resolution -- make sure 0 is included
	local search_x = torch.range( pose_guess.x-ddx, pose_guess.x+ddx, map.resolution )
	--
	local dy = search_amount.y or .25
	local ddy = math.floor(dy*map.inv_resolution)*map.resolution -- make sure 0 is included
	local search_y = torch.range( pose_guess.y-ddy, pose_guess.y+ddy, map.resolution )
	--
	local a  = search_amount.a  or 5*DEG_TO_RAD
	local da = search_amount.da or 1*DEG_TO_RAD
	local dda = math.floor(a/da)*da -- make sure 0 is included
	local search_a = torch.range( pose_guess.a-dda, pose_guess.a+dda, da )

	-- Perform the match
	slam.set_resolution(map.resolution)
	slam.set_boundaries(map.bounds_x[1],map.bounds_y[1],map.bounds_x[2],map.bounds_y[2])
	local likelihoods, max =
		slam.match( map.omap, laser_points, search_a, search_x, search_y, prior or 200 )

	--print("dd",ddx,ddy,dda)
	--print(pose_guess)
	--util.ptable( max )
	--util.ptorch( search_x )
	--print("size",search_x:size(1),search_y:size(1),search_a:size(1))

	local matched_pose = vector.pose{search_x[max.x],search_y[max.y],search_a[max.a]}
	--
	return matched_pose, max.hits
end

libMap.export = function( map, filename, kind )
	assert(map:isContiguous(),'Map must be contiguous!')
	local sz = map:size()
	local ptr, n_el = map:storage():pointer(), #map:storage()
	-- Export for MATLAB
	local f = io.open(filename, 'w')
	f:write( tostring(carray.double({sz[1],sz[2]}) ) )
	local data
	if kind=='byte' then
		data = carray.byte( ptr, n_el)
		--print(n_el,'n_el',kind,sz[1],sz[2],#data)
	else
		data = carray.double( ptr, n_el)
	end
	f:write( tostring( data ) )
	f:close()
end

return libMap
