local plugins = {}
local T = require'Transform'


local function get_world_torso()
	local rpy = Body.get_rpy()
	local uComp = mcm.get_stance_uTorsoComp()
	uComp[3] = 0
	local torso0 = pose_global(uComp, mcm.get_status_bodyOffset())
	local pose = wcm.get_robot_pose()
	local bh = mcm.get_walk_bodyHeight()
	local qHead = Body.get_head_position()
	--local torso = {torso0.x, torso0.y, bh, rpy[1], rpy[2], torso0.a}
	local torsoG = pose_global(torso0, pose)
	local global = {torsoG.x, torsoG.y, bh, rpy[1], rpy[2], torsoG.a}
	return global
end

-- Open Pull Door
--[[
x: World x coordinate of the handle
y: World y coordinate of the handle
z: World z coordinate of the handle
yaw: Relative yaw of the door with respect to the frame. 0 is closed. (+) is opening.
hinge: Relative y coordinate of the hinge. (+) is hinge to the left of the handle
roll: Relative roll of the door handle. 0 is open, +/- pi/2 is closed

Phase is the yaw on the door hinge
--]]
function plugins.pull_door(m)
	-- Assume right hand for now
	-- Assume hinge to the right of the handle
	-- Assume Relative to torso
	local phase
	-- Need to know where the hinge is?
	local tfHinge = T.trans(0, m.hinge, 0) * T.rotZ(-m.yaw) * T.trans(-m.x, -m.y, m.z)
	-- Know where the handle is
	local tfHandle = T.transform6D({m.x, m.y, m.z, 0, 0, m.yaw})
	-- Roll for the grip
	local tfGrip = T.rotX(m.roll)
	local tfHandGoal = tfHandle * tfGrip
	print(T.tostring(tfHandGoal))
end

function plugins.test()
  m ={}
  m.x = .6
  m.y = -.21
  m.z = 0
  m.yaw = 0
  m.hinge = 1
  m.roll = -math.pi/2
  plugins.pull_door(m)
end

return plugins
