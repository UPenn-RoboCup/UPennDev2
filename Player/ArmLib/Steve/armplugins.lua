local plugins = {}
-- Allowed to use Body here
local Body = require'Body'
local T = require'Transform'
local tr6D = require'Transform'.transform6D
local movearm = require'movearm'
local util = require'util'
local vector = require'vector'
local vnorm = vector.norm

local lPlanner = movearm.lPlanner
local rPlanner = movearm.rPlanner

local function get_vw(tfObject, fkArm)
	assert(tfObject)
	assert(fkArm)
	local invArm = T.inv(fkArm)
	local here = invArm * tfObject
	local dp = T.position(here)
	local drpy = T.to_rpy(here)
	return vector.new{dp[1], dp[2], dp[3], unpack(drpy)}, vnorm(dp), vnorm(drpy)
end


function plugins.turnvalve(m)

end

function plugins.pushdoor(m)

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

Assume right hand for now
Assume hinge to the right of the handle
Assume Relative to torso

--]]
function plugins.pulldoor(m)

	-- TODO: Search over the roll to keep smooth
	local yawGoal = math.pi / 6
	local tfHinge = T.trans(0, m.hinge, 0) * T.rotZ(m.yaw) * T.trans(m.x, m.y, m.z)
	local pHinge = T.position(tfHinge)
	tfHinge = T.trans(unpack(pHinge))
	local tfHandle0 = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)

	local qRArm0 = Body.get_rarm_command_position()
	--local qArmHandle0 = rPlanner:find_shoulder(tfHandle0, qRArm0, {1,0,0})
	--print()
	--print('qArmHandle0', qArmHandle0)

	--[[
	local tfHingeGoal = T.trans(0, m.hinge, 0) * T.rotZ(yawGoal) * T.trans(m.x, m.y, m.z)
	local pHingeGoal = T.position(tfHingeGoal)
	tfHingeGoal = T.trans(unpack(pHingeGoal))
	local tfHandleGoal = tfHingeGoal * T.rotZ(yawGoal) * T.trans(0,-m.hinge,0)
	--]]
	--get_vw(tfObject, fkArm)
	-- I know that drpy.yaw should be scaled, and dx, dy
	-- Technically there should be some manifold distance metric on so(3) paths
	--yawGoal - m.yaw

	local configL = {
		via='jacobian_velocity',
	}
	local configR = {
		tr=tfHandle0, timeout=10,
		via='jacobian_waist_preplan', weights = {1,0,0}
	}

	local lstatus, rstatus = coroutine.yield(movearm.goto(configL, configR))

	-- TODO: Add a timeout here for reaching the handle...
	repeat
		lstatus, rstatus = coroutine.yield({}, {})
		--print('lstatus, rstatus', lstatus, rstatus)
	until rstatus=='dead'
	print('At the handle')


	local vw, distp, dista
	-- Next stage
	local configL = {
		via='jacobian_velocity',
	}
	local configR = {
		via='jacobian_velocity',
	}
	coroutine.yield(movearm.goto(configL, configR))


	local n_ph = 50
	local ph0 = math.ceil((m.yaw / yawGoal) * n_ph)
	local ph = ph0
	repeat
		-- Find the arm
		local qRArm = Body.get_rarm_position()
		local fkRArm = rPlanner.forward(qRArm)

		m.ph = ph
		m.yaw = (ph / n_ph) * yawGoal
		-- Know where the handle is
		local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)
		local pHandle = T.position(tfHandle)
		-- Roll for the grip
		local tfGrip = T.rotX(m.roll)
		local tfHandGoal = tfHandle * tfGrip
		vw, distp, dista = get_vw(tfHandle, fkRArm)

		--print('components', components[1], components[2]*RAD_TO_DEG)
		if distp<0.02 and dista<3*DEG_TO_RAD then
			ph = ph + 1
			print(ph, 'pHandle', vector.new(pHandle))
			qLArm, qRArm = coroutine.yield(
				{},
				{scaled_vw, {1,1,0}}
			)
		else
			qLArm, qRArm = coroutine.yield({},{vw})
		end

	until ph>=n_ph
	return false
end

function plugins.gen(name, model)
	if not name then return end
	if not plugins[name] then return end
	local pco =  coroutine.create(plugins[name], model)
	local status, lco, rco = coroutine.resume(pco, model)
	if not status then print('pco', status, lco) end
	return pco, lco, rco
end

function plugins.test()


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
