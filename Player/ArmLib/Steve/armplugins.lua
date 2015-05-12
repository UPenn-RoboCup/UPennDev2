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
	local yawGoal = 30*DEG_TO_RAD
	local qWaistGuess = vector.new{45, 0}*DEG_TO_RAD

	local tfHinge = T.trans(0, m.hinge, 0) * T.rotZ(m.yaw) * T.trans(m.x, m.y, m.z)
	local pHinge = T.position(tfHinge)
	tfHinge = T.trans(unpack(pHinge))
	local tfHandle0 = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)

	local qRArm0 = Body.get_rarm_command_position()
	local qWaist0 = Body.get_waist_command_position()
	local qArmHandle0 = rPlanner:find_shoulder(tfHandle0, qRArm0, {1,0,0}, qWaist0)
	qArmHandle0 = qArmHandle0 or rPlanner.zeros
	local qArmHandle0 = rPlanner:find_shoulder(tfHandle0, qArmHandle0, {1,0,0}, qWaistGuess)
	vector.new(qArmHandle0)

	--[[
	print()
	print('pulldoor | qWaistGuess', qWaistGuess*RAD_TO_DEG)
	print(tfHandle0)
	if qArmHandle0 then
		print('pulldoor | qArmHandle0',qArmHandle0*RAD_TO_DEG)
	else
		print('pulldoor | qArmHandle0 no good')
	end
	--]]

	--get_vw(tfObject, fkArm)
	-- I know that drpy.yaw should be scaled, and dx, dy
	-- Technically there should be some manifold distance metric on so(3) paths
	--yawGoal - m.yaw

	local configL1 = false -- not moving
	local configR1 = {
		tr=tfHandle0, timeout=10,
		via='jacobian_waist_preplan', weights = {1,0,0},
		qWaistGuess = qWaistGuess,
		qArmGuess = qArmHandle0
	}

	local lstatus, rstatus = coroutine.yield(movearm.goto(configL1, configR1))

	-- TODO: Add a timeout here for reaching the handle...
	repeat
		lstatus, rstatus = coroutine.yield({}, {})
		--print('lstatus, rstatus', lstatus, rstatus)
	until rstatus=='dead'
	print('At the handle')
	--if true then return end

	local qWaistGuess1 = vector.new{-30, 0}*DEG_TO_RAD

	local vw, distp, dista
	-- Next stage
	local configL2 = false
	local configR2 = {
		via='jacobian_waist_velocity',
		vw = {0,0,0, 0,0,0},
		qWaistGuess = qWaistGuess1
	}
	coroutine.yield(movearm.goto(configL2, configR2))

	print('Running next motion...')
	local n_ph = 40
	local ph0 = math.ceil((m.yaw / yawGoal) * n_ph)
	local ph = ph0
	local intra_ph_count = 0
	local intra_ph_timeout = 15 * rPlanner.hz
	local pHandle
	repeat
		intra_ph_count = intra_ph_count + 1
		assert(intra_ph_count < intra_ph_timeout,
			string.format("pulldoor | intra_ph_timeout %d/%d", ph, n_ph))
		-- Find the arm
		local qRArm = Body.get_rarm_position()
		local qWaist = Body.get_waist_position()
		local fkRArm = rPlanner.forward(qRArm, qWaist)

		m.ph = ph
		m.yaw = (ph / n_ph) * yawGoal
		-- Know where the handle is
		local tfHandle = tfHinge * T.rotZ(m.yaw) * T.trans(0,-m.hinge,0)
		pHandle = T.position(tfHandle)
		-- Roll for the grip
		local tfGrip = T.rotX(m.roll)
		local tfHandGoal = tfHandle * tfGrip
		vw, distp, dista = get_vw(tfHandle, fkRArm)

		--print(distp, 'vw', vector.new(vw))
		if distp<0.02 and dista<3*DEG_TO_RAD then
			ph = ph + 1
			intra_ph_count = 0
			print(ph, 'pHandle', vector.new(pHandle))
			qLArm, qRArm = coroutine.yield(
				{},
				{vw, {1,1,0}}
			)
		else
			qLArm, qRArm = coroutine.yield({},{vw})
		end

	until ph>=n_ph

	print('Door opened!')

	-- Now change hands
	print('pHandle', pHandle)
	local tfEdgeGoal = T.trans(0.36,pHandle[2]+0.05, 0.15) * T.rotZ(-90*DEG_TO_RAD)
	print('tfEdgeGoal')
	print(tfEdgeGoal)
	local qLArm = Body.get_larm_command_position()
	local qWaist = Body.get_waist_command_position()
	local tfL = lPlanner.forward(qLArm, qWaist)
	print('tfL')
	print(T.tostring(tfL))
	--
	local configL3 = {
		tr=tfEdgeGoal, timeout=20,
		via='jacobian_preplan', weights = {1,0,0},
		--qWaistGuess = qWaistGuess1
	}
	local configR3 = false
	coroutine.yield(movearm.goto(configL3, configR3))

	repeat
		lstatus, rstatus = coroutine.yield({}, {})
		--print('lstatus, rstatus', lstatus, rstatus)
	until lstatus=='dead'
	print('Two hand hold')

	return
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
