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
-- Make the metatable
--local mt = {}

local function pose_to_map_index(map,pose)
	--print('Pose:',pose)
	--print('Map size:',map.size,'meters')
	--print('Map offset:',map.offset,'meters')
	local inv_pose = pose * map.inv_resolution
	--print('Inverse Pose:',inv_pose)
	local map_sz = vector.new{map.cost:size(1),map.cost:size(2)}
	--print('Map size',map_sz)
	local map_pose = map_sz/2 + inv_pose
	--print('map_pose',map_pose)
	i = math.max(math.min(math.ceil(map_pose[1]),map_sz[1]),1)
	j = math.max(math.min(math.ceil(map_pose[2]),map_sz[2]),1)
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
	-- Make the double of the cost map
	map.cost = torch.DoubleTensor(ncolumns,nrows)
	-- Make 1 free space, and max+1 the max?
	-- TODO: Should be log odds... For now, whatever
	map.cost:copy(img_t):mul(-1):add(max+1)
	-- Save the map byte image
	map.map = img_t

	-- Give the table
	--return setmetatable(map, mt)
	map.new_goal = libMap.new_goal
	map.new_path = libMap.new_path
	map.render = libMap.render
	return map
	
end

-- Compute the cost to go to the goal
libMap.new_goal = function( map, goal )
	assert(map.cost,'You must open a map first!')
	--print('GOAL')
	local i, j = pose_to_map_index(map,goal)
	--print('GOAL IDX',i,j)
	map.cost_to_go = dijkstra.matrix( map.cost, i, j )
end

-- Compute a path to the goal
libMap.new_path = function( map, start )
	assert(map.cost_to_go,'You must set a goal first!')
	--print('START')
	local i, j = pose_to_map_index(map,start)
	--print('START IDX',i,j)
	local i_path, j_path = dijkstra.path( map.cost_to_go, map.cost, i, j )
	return index_path_to_pose_path(map,i_path,j_path)
end

libMap.render = function( map, fmt )
	-- Export in grayscale
	local w, h = map.cost:size(1), map.cost:size(2)
	if fmt=='jpg' or fmt=='jpeg' then
		return jpeg.compress_gray( map.map:storage():pointer(), w, h )
	elseif fmt=='png' then
		return png.compress( map.map:storage():pointer(), w, h, 1 )
	end
end

-- Metatable methods
--mt.new_goal = libMap.new_goal
--mt.new_path = libMap.new_path

return libMap
