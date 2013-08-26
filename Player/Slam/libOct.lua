-- libOct
-- (c) 2013 Stephen McGill
-- Octomap processing and helpers

require'torch'
torch.Tensor = torch.DoubleTensor
local libTrig = require'libTrig'
local tutil = require'tutil'
local octomap = require'octomap'

local libOct = {}
libOct.max_npoints = 2000
libOct.points_xyz = torch.Tensor(libOct.max_npoints,4):zero()
libOct.debug_prefix = 'libOct | '

libOct.subsample = function( points )
	--------------
	-- Input checking
	local nPoints = points:size(1)
	if nPoints>libOct.max_npoints or nPoints<5 then
		print(libOct.debug_prefix,'Bad points',nPoints)
		return
	end
	libOct.points_xyz:resize( nPoints, 4 )
	libOct.points_xyz:copy( points )
	--------------
	
	--[[
	--------------
	-- Prune for the z constraints
	local zs = libOct.points_xyz:select(2,3)
	tutil.band_mask_key_points(
	zs,
	-1,0, -- 0 to 2 meters off the ground
	libOct.points_xyz
	)
	--------------
	--]]
	
	--------------
	-- Prune for the y constraints
	local ys = libOct.points_xyz:select(2,2)
	tutil.band_mask_key_points(
	ys,
	-.5,.5,
	libOct.points_xyz
	)
	--------------
	
end

libOct.update = function()
	--------------
	-- Add the XYZ points to the octomap cloud
	octomap.add_scan( libOct.points_xyz )
	
	--[[
	local mid = libOct.points_xyz:size(1) / 2
	local str = string.format('\n\nMidpoint:   %6.4f  %6.4f  %6.4f\n',
	libOct.points_xyz[mid][1],libOct.points_xyz[mid][2],libOct.points_xyz[mid][3]
	)
	io.write(str)
	--]]
end

libOct.get_tree = function()
	octomap.get_pruned_data()
	octomap.save_tree('log_tree.bt')
end

-- Yield the library object
return libOct
