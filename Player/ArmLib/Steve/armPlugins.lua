local plugins = {}
local T = require'Transform'
local util = require'util'

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

	-- TODO: Search over the roll to keep smooth


	local ph1 = 10
	local yawGoal = math.pi / 4
	local ph0 = math.ceil((m.yaw / yawGoal) * ph1)

	local tfHinge = T.trans(0, m.hinge, 0) * T.rotZ(m.yaw) * T.trans(m.x, m.y, m.z)
	local pHinge = T.position(tfHinge)
	--print('pHinge', pHinge)
	tfHinge = T.trans(unpack(pHinge))

	for ph = ph0, ph1 do
		m.ph = ph
		m.yaw = (ph / ph1) * yawGoal
		--print('m.yaw', m.yaw)
		-- Know where the handle is
		local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)
		local pHandle = T.position(tfHandle)
		-- Roll for the grip
		local tfGrip = T.rotX(m.roll)
		local tfHandGoal = tfHandle * tfGrip
		m = coroutine.yield(tfHandGoal, m)
	end
end

function plugins.gen(name)
	if not name then return end
	if not plugins[name] then return end
	return coroutine.create(plugins[name])
end

function plugins.test()
  m ={}
  m.x = .6
  m.y = -.21
  m.z = 0
  m.yaw = 0
  m.hinge = -1
  m.roll = -math.pi/2
	m.hand = 'right'

	local f = coroutine.create(plugins.pull_door)
	local ok, tfHandGoal
	repeat
		local ok, tfHandGoal, ph = coroutine.resume(f, m)
		print(ok)
		if ok and ph then
			print(T.tostring(tfHandGoal))
		else
			print(tfHandGoal)
		end
		--util.ptable(ph or {})
	until not ok
end

return plugins
