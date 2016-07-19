--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local T = require'Transform'
require'rcm'

local t_entry, t_update, t_finish
local timeout = 30.0

-- left and right will both request waist positions potentially
local lco, rco
local okL, qLWaypoint, qLWaist
local okR, qRWaypoint, qRWaist
local sequence, s, stage

function state.entry()
	io.write(state._NAME, ' Entry\n')
	local t_entry_prev = t_entry
	t_entry = Body.get_time()
	t_update = t_entry



eef_vector = rcm.get_RRT_eef()

grasping_eef = {}

len = 0.00
dim = 0.05
table.insert(grasping_eef, {
	right = {
		timeout=10,
		via='jacobian_preplan',
--		tr={0.4338 - 0.03 -0.08, -0.277838, -0.039592, -0.003704, -0.055883, 0.000970}, --norminal
--		tr={0.44, -0.277838, -0.039592, -0.003704, -0.055883, 0.000970}, --norminal
--		tr={0.44, -0.277838 -0.05, -0.039592, -0.003704, -0.055883, 45*DEG_TO_RAD},   --45deg
		tr=eef_vector,

		--qArmGuess = vector.new{-15, 60, 90, -120, -80, -70, 0}*DEG_TO_RAD,
		weights = {1,1,1},
	},
	left = false	
})



	sequence = {unpack(grasping_eef)}

	-- When to reset?
	s = 1
	stage = sequence[s]
	lco, rco = movearm.goto(stage.left, stage.right)

	-- Check for no motion
	okL = type(lco)=='thread' or lco==false
	okR = type(rco)=='thread' or rco==false
	qLWaypoint = nil
	qRWaypoint = nil
	qLWaistpoint = nil
	qRWaistpoint = nil

end

function state.update()
	--  io.write(state._NAME,' Update\n')
	local t  = Body.get_time()
	local dt = t - t_update
	t_update = t
	--if t-t_entry > timeout then return'timeout' end

	if not stage then return'done' end

	local lStatus = type(lco)=='thread' and coroutine.status(lco) or 'dead'
	local rStatus = type(rco)=='thread' and coroutine.status(rco) or 'dead'

	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()
	local qWaist = Body.get_safe_waist_position()
	if lStatus=='suspended' then
		okL, qLWaypoint, qLWaistpoint = coroutine.resume(lco, qLArm, qWaist)
	end
	if rStatus=='suspended' then
		okR, qRWaypoint, qRWaistpoint = coroutine.resume(rco, qRArm, qWaist)
	end

	-- Check if errors in either
	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint, lco, lStatus)
		print(state._NAME, 'R', okR, qRWaypoint, rco, rStatus)
		-- Safety
		local qcLArm = Body.get_larm_command_position()
		local qcRArm = Body.get_rarm_command_position()
		local qcWaist = Body.get_safe_waist_command_position()
		--
		Body.set_larm_command_position(qcLArm)
		Body.set_rarm_command_position(qcRArm)
		Body.set_safe_waist_command_position(qcWaist)
		--
		return'teleopraw'
	end

	if type(qLWaypoint)=='table' then
		Body.set_larm_command_position(qLWaypoint)
	end
	if type(qRWaypoint)=='table' then
		Body.set_rarm_command_position(qRWaypoint)
	end

	--[[
	if qLWaistpoint and qRWaistpoint then
		print('Conflicting Waist')
	elseif qLWaist then
		Body.set_safe_waist_command_position(qLWaistpoint)
	elseif qRWaist then
		Body.set_safe_waist_command_position(qRWaistpoint)
	end
	--]]
	-- Zero the waist
	--[[
  local qWaist = Body.get_safe_waist_command_position()
  local qWaist_approach, doneWaist = util.approachTol(qWaist, {0,0}, {2 * DEG_TO_RAD, 2 * DEG_TO_RAD}, dt, {1*DEG_TO_RAD, 1*DEG_TO_RAD})
  Body.set_safe_waist_command_position(qWaist_approach)
	--]]

	-- Check if done
	if lStatus=='dead' and rStatus=='dead' then
		-- Goto the nextitem in the sequnce
		s = s + 1
		local oldstage = stage
		stage = sequence[s]
		if stage then

			local qcLArm = Body.get_larm_command_position()
			local qcRArm = Body.get_rarm_command_position()
			local qcWaist = Body.get_safe_waist_command_position()
			local fkL = movearm.lPlanner.forward(qcLArm, qcWaist)
			local fkR = movearm.rPlanner.forward(qcRArm, qcWaist)

			if oldstage.left then
				local dtrL = oldstage.left.tr * T.inv(fkL)
				print('dtrL')
				print(dtrL)
			end
			if oldstage.right then
				local dtrR = oldstage.right.tr * T.inv(fkR)
				print('dtrR')
				print(dtrR)
			end

			print('Next sequence:', s, stage)
			lco, rco = movearm.goto(stage.left, stage.right)
			-- Check the tr offsets from expected
			okL = type(lco)=='thread' or lco==false
			okR = type(rco)=='thread' or rco==false
			return'finetune'
		else

        Grip_close = {0.496719, 0.458369, 0.50132}
        Grip_open = {-0.496719, -0.458369, -0.50132}
        Body.set_rgrip_command_position(Grip_close)
			return'done'
		end
	end

end

function state.exit()
	print(state._NAME..' Exit')

end

return state
