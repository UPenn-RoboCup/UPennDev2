-- libDetect
-- (c) 2014 Stephen McGill
-- General Detection methods
local libDetect = {}
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor
local vector = require'vector'
local q = require'quaternion'
local util = require 'util'
local T = require'libTransform'
-- Intimate relationship with SHM maybe
require'wcm'

local function is_1d_semicircle (xs, ys, nps)
	if nps<8 then return("FAIL: nps: "..nps) end
	local first = vector.new{xs[1],ys[1]}
	local last = vector.new{xs[nps],ys[nps]}
	local diff_chord = last - first
	local diameter = vector.norm(diff_chord)
	if diameter>.2 then
		return string.format("FAIL: diameter (%f)",diameter)
	end
	local assumed_radius = diameter/2
	-- Phantom center
	local center_phantom = first+(diff_chord/2)
	-- Sample approximately 10 points
	local inc = math.ceil(nps / 10)
	local sum = diameter
	local radii_phantom = {}
	for imid=2,nps-1,inc do
		local candidate = vector.new{xs[imid],ys[imid]}
		local radius_phantom = vector.norm(candidate-center_phantom)
		sum = sum + radius_phantom
		table.insert(radii_phantom,radius_phantom)
	end
	local ninspect = #radii_phantom+2
	local radius_phantom = sum / ninspect
	if radius_phantom<.01 or radius_phantom>.1 then
		return string.format("FAIL: radius (%f)",radius_phantom)
	end
	local sum_sq = 0
	for _,r in ipairs(radii_phantom) do
		local diff = r-radius_phantom
		sum_sq = sum_sq + diff^2
	end
	local variance_phantom = sum_sq/(ninspect - 1)
	local stddev_phantom = math.sqrt(variance_phantom)
	local ratio = stddev_phantom / radius_phantom
	if ratio>0.2 then
		return string.format("FAIL: deviation ratio (%f)",ratio)
	end
	-- This is a semicircle, so form the description
	center_phantom[3] = 0
	vector.pose(center_phantom)
	return {pose=center_phantom,radius=radius_phantom}
end

-- Give a channel that has lidar return information
-- Give a Connected compoments segmentation obj
function libDetect.lidar_circles (ch, cc)
	-- Run the components if not done yet
	cc = cc or slam.connected_components(ch.raw,0.05)
	local pose = wcm.get_robot_pose()
	local pts_x = ch.points:select(2,1)
	local pts_y = ch.points:select(2,2)
	local circles = {}
	for i,c in ipairs(cc) do
		local nps = c.stop - c.start + 1
		local xs = pts_x:narrow(1,c.start,nps)
		local ys = pts_y:narrow(1,c.start,nps)
		local circle = is_1d_semicircle(xs,ys,nps)
		if type(circle)=='table' then
			circle.pose = util.pose_global(circle.pose,pose)
			table.insert(circles, circle)
		end
	end
	-- Update the shared memory
	-- TODO: Multiple circles?
	if #circles>0 then
		wcm.set_ball_pose( circles[1].pose )
		wcm.set_ball_t(Body.get_time())
	end
end

return libDetect
