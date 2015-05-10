local plugins = {}
local T = require'Transform'
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
--]]
function plugins.pulldoor(m)
	-- Assume right hand for now
	-- Assume hinge to the right of the handle
	-- Assume Relative to torso

	local alpha = 1
	local yawGoal = math.pi / 6

	-- TODO: Search over the roll to keep smooth

	local tfHinge = T.trans(0, m.hinge, 0) * T.rotZ(m.yaw) * T.trans(m.x, m.y, m.z)
	local pHinge = T.position(tfHinge)
	tfHinge = T.trans(unpack(pHinge))
	local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)

	local tfHinge = T.trans(0, m.hinge, 0) * T.rotZ(yawGoal) * T.trans(m.x, m.y, m.z)
	local pHinge = T.position(tfHinge)
	tfHinge = T.trans(unpack(pHinge))
	local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)

	--get_vw(tfObject, fkArm)


	local vw, distp, dista
	local qLArm, qRArm = coroutine.yield()
	-- TODO: Add a timeout here for reaching the handle...
	repeat
		local fkRArm = rPlanner.forward(qRArm)
		vw, distp, dista = get_vw(tfHandle, fkRArm)
		qLArm, qRArm = coroutine.yield(vw)
	until distp<0.01 and dista<2*DEG_TO_RAD
	print('At the handle')

	local n_ph = 50
	local ph0 = math.ceil((m.yaw / yawGoal) * n_ph)
	local ph = ph0
	repeat
		m.ph = ph
		m.yaw = (ph / n_ph) * yawGoal
		-- Know where the handle is
		local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)
		local pHandle = T.position(tfHandle)
		-- Roll for the grip
		local tfGrip = T.rotX(m.roll)
		local tfHandGoal = tfHandle * tfGrip
		local fkRArm = rPlanner.forward(qRArm)
		vw, distp, dista = get_vw(tfHandle, fkRArm)

		-- Scale by the phase
		local scaled_vw = alpha * vw

		--print('components', components[1], components[2]*RAD_TO_DEG)
		if distp<0.02 and dista<3*DEG_TO_RAD then
			ph = ph + 1
			print(ph, 'pHandle', vector.new(pHandle))
		end
		qLArm, qRArm = coroutine.yield(scaled_vw, weights, qArmGuess)
	until ph>=n_ph
	print('Done routine')
end

function plugins.gen(name, model)
	if not name then return end
	if not plugins[name] then return end
	local pco =  coroutine.create(plugins[name], model)
	local status, msg = coroutine.resume(pco, model)
	if not status then print('pco', status, msg) end
	return pco
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
