local plugins = {}
local T = require'Transform'
local movearm = require'movearm'
local util = require'util'
local vector = require'vector'
local vnorm = vector.norm

local lPlanner = movearm.lPlanner
local rPlanner = movearm.rPlanner

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

	-- TODO: Search over the roll to keep smooth
	local function get_vw(tfObject, fkArm)
		assert(tfObject)
		assert(fkArm)
		local invArm = T.inv(fkArm)
		local here = invArm * tfObject
		local dp = T.position(here)
		local drpy = T.to_rpy(here)
		local components = {vnorm(dp), vnorm(drpy)}
		--print('components', unpack(components))
		return {dp[1], dp[2], dp[3], unpack(drpy)}, components
	end

	local n_ph = 10
	local yawGoal = math.pi / 4
	local ph0 = math.ceil((m.yaw / yawGoal) * n_ph)

	local tfHinge = T.trans(0, m.hinge, 0) * T.rotZ(m.yaw) * T.trans(m.x, m.y, m.z)
	local pHinge = T.position(tfHinge)
	tfHinge = T.trans(unpack(pHinge))
	print('tfHinge')
	print(tfHinge)

	local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)
	print('tfHandle')
	print(tfHandle)

	-- Find where we are
	local components
	local vw
	local qLArm, qRArm = coroutine.yield()
	-- TODO: Add a timeout here for reaching the handle
	repeat
		local fkLArm = lPlanner.forward(qLArm)
		local fkRArm = rPlanner.forward(qRArm)
		vw, components = get_vw(tfHandle, fkRArm)
		assert(vw)
		assert(components)
		qLArm, qRArm = coroutine.yield(vw)
		--print('dp', unpack(dp))
		--print('drpy', vector.new(drpy)*RAD_TO_DEG)
		--print('components', components[1], components[2]*RAD_TO_DEG)
	until components[1]<0.01 and components[2]<2*DEG_TO_RAD
	print('At the handle')

	--for ph = ph0, n_ph do
	local ph = ph0
	repeat
		m.ph = ph
		m.yaw = (ph / n_ph) * yawGoal
		--print('m.yaw', m.yaw)
		-- Know where the handle is
		local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)
		local pHandle = T.position(tfHandle)
		-- Roll for the grip
		local tfGrip = T.rotX(m.roll)
		local tfHandGoal = tfHandle * tfGrip
		local fkLArm = lPlanner.forward(qLArm)
		local fkRArm = rPlanner.forward(qRArm)
		vw, components = get_vw(tfHandle, fkRArm)
		--print('components', components[1], components[2]*RAD_TO_DEG)
		if components[1]<0.01 and components[2]<2*DEG_TO_RAD then
			ph = ph + 1
			print(ph, 'pHandle', vector.new(pHandle))
		end

		qLArm, qRArm = coroutine.yield(vw)
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
