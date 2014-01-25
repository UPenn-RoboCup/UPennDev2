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

local function pose_to_map_index(map,pose)
	local inv_pose = pose * map.inv_resolution
	local map_sz = vector.new{map.cost:size(1),map.cost:size(2)}
	local map_pose = map_sz/2 + inv_pose
	local i = math.max(math.min(math.ceil(map_pose[1]),map_sz[1]),1)
	local j = math.max(math.min(math.ceil(map_pose[2]),map_sz[2]),1)
	return i, j
end

local function index_path_to_pose_path(map,i_path,j_path)
	-- Should return a lua table, not a torch object
	local pose_path = {}
	local i_mid, j_mid = map.cost:size(1)/2, map.cost:size(2)/2
	local npath = i_path:size(1)
	-- Go in reverse order: last item is is the next point in the path
	-- Easy pop operation in lua
	for p=npath,1,-1 do
		local x = (i_path[p] - i_mid) * map.resolution
		local y = (j_path[p] - j_mid) * map.resolution
		table.insert(pose_path,vector.pose{x,y,0})
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
	-- Save the map byte image
	return cost
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
	map.size = map.resolution*vector.new{ncolumns,nrows}
	-- Second comment are x and y offsets
	local m_offset = comments[2]:gmatch("%S+")
	local header = m_offset()
	assert(header=='#offset','Bad offset header')
	local x_off = tonumber(m_offset())
	assert(x_off,'Bad X offset')
	local y_off = tonumber(m_offset())
	assert(y_off,'Bad Y offset')
	map.offset = vector.new{x_off,y_off}
	
	-- Read the actual map image
	local img_str = f_img:read('*all')
	
	-- Close the map image file
	f_img:close()

	-- Make the Byte tensor to container the PGM bytes of the map
	local img_t
	if f_type=='P5' then
		-- Grayscale
		assert(#img_str==ncolumns*nrows,'Bad Greyscale resolution check!')
		img_t = torch.ByteTensor(ncolumns,nrows)
		-- Copy the pgm img string to the tensor
		cutil.string2storage(img_str,img_t:storage())
	elseif f_type=='P6' then
		-- RGB
		assert(#img_str==ncolumns*nrows*3,'Bad RGB resolution check!')
		local rgb_t = torch.ByteTensor(ncolumns,nrows,3)
		cutil.string2storage(img_str,rgb_t:storage())
		-- Just the R channel
		img_t = rgb_t:select(3,1):clone()
	else
		error('Unsupported!')
	end

	-- Save the map byte image
	map.map = img_t
	-- Make the double of the cost map
	map.cost = map_to_cost(img_t)

	-- Give the table
	--return setmetatable(map, mt)
	map.new_goal = libMap.new_goal
	map.new_path = libMap.new_path
	map.grow = libMap.grow
	map.render = libMap.render
	return map
	
end

-- Grow the costs so that the robot will not hit anything
libMap.grow = function( map, radius )
	assert(map.cost,'You must open a map first!')
	radius = math.ceil( (radius or .4) * map.inv_resolution )
	-- Replace the cost map with the grown map
	map.cost = slam.grow_map(map.cost, radius)
end

-- Compute the cost to go to the goal
libMap.new_goal = function( map, goal )
	assert(map.cost,'You must open a map first!')
	local i, j = pose_to_map_index(map,goal)
	map.cost_to_go = dijkstra.matrix( map.cost, i, j )
end

-- Compute a path to the goal
libMap.new_path = function( map, start )
	assert(map.cost_to_go,'You must set a goal first!')
	local i, j = pose_to_map_index(map,start)
	local i_path, j_path = dijkstra.path( map.cost_to_go, map.cost, i, j )
	return index_path_to_pose_path(map,i_path,j_path)
end

libMap.render = function( map, fmt )
	-- Export in grayscale
	local w, h = map.map:size(1), map.map:size(2)
	if fmt=='jpg' or fmt=='jpeg' then
		return jpeg.compress_gray( map.map:storage():pointer(), w, h )
	elseif fmt=='png' then
		return png.compress( map.map:storage():pointer(), w, h, 1 )
	end
end

libMap.export = function( map, filename )
	assert(map:isContiguous(),'Map must be contiguous!')
	-- Export for MATLAB
	local f = io.open(filename, 'w')

	local sz = map:size()
	f:write( tostring(carray.double{sz[1],sz[2]}) )

	local ptr, n_el = map:storage():pointer(), #map:storage()
	local arr = carray.double(ptr, n_el)

	f:write( tostring(arr) )
	f:close()
end

return libMap
