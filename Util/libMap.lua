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
-- Make the metatable
--local mt = {}

-- import a map
libMap.open_map = function( map_filename )
	local map = {}
	local f_img = io.open(map_filename,'r')
	local f_type = f_img:read('*line')
	--print('NetPBM type',f_type)
	-- Assume PGM greyscale binary for now
	assert(f_type=='P5','PGM binary file support only! (P5 Header)')
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
	print('header',header)
	assert(header=='#resolution','Bad resolution header')
	map.map_res = tonumber(m_res())
	assert(map.map_res,'Bad resolution')
	-- Second comment are x and y offsets
	local m_offset = comments[2]:gmatch("%S+")
	local header = m_offset()
	assert(header=='#offset','Bad offset header')
	local x_off = tonumber(m_offset())
	assert(x_off,'Bad X offset')
	local y_off = tonumber(m_offset())
	assert(y_off,'Bad Y offset')
	map.map_offset = vector.new{x_off,y_off}
	
	-- Read the actual map image
	local img_str = f_img:read('*all')
	
	-- Close the map image file
	f_img:close()
	
	-- Check the resolution
	assert(#img_str==ncolumns*nrows,'Bad resolution check!')
	
	-- Make the Byte tensor to container the PGM bytes of the map
	local img_t = torch.ByteTensor(ncolumns,nrows)
	-- Copy the pgm img string to the tensor
	cutil.string2storage(img_str,img_t:storage())
	-- Make the double of the cost map
	map.cost_map = torch.DoubleTensor(ncolumns,nrows)
	-- Make 1 free space, and max+1 the max?
	-- TODO: Should be log odds... For now, whatever
	map.cost_map:copy(img_t):mul(-1):add(max+1)

	-- Give the table
	--return setmetatable(map, mt)
	map.new_goal = libMap.new_goal
	map.new_path = libMap.new_path
	return map
	
end

-- Compute the cost to go to the goal
libMap.new_goal = function( map, goal )
	assert(map.cost_map,'You must open a map first!')
	map.cost_to_go = dijkstra.matrix(map.cost_map, goal[1], goal[2])
end

-- Compute a path to the goal
libMap.new_path = function( map, start )
	assert(map.cost_to_go,'You must set a goal first!')
	return dijkstra.path(map.cost_to_go, map.cost_map, start[1], start[2]);
end

--[[
local function update_wheel_velocity( self )
	local max_wheel_velocity = 4000
	local t_now = unix.time()
	local t_diff = t_now - self.last_update
	self.last_update = t_now
	
	-- Attenuate the velocity if not just commanded
	if self.mode == 'waypoint' then
		self:update_waypoint_velocity()
	else
		if self.t_commanded then
			self.t_commanded = false
		else
			self.vx = self.vx * self.attenuation*t_diff
			self.vy = self.vx * self.attenuation*t_diff
		end
	end

	-- TODO: better conversion factor
	Body.set_lwheel_velocity(self.vx)
	Body.set_rwheel_velocity(self.vy)
end
--]]

-- Metatable methods
--mt.new_goal = libMap.new_goal
--mt.new_path = libMap.new_path

return libMap
