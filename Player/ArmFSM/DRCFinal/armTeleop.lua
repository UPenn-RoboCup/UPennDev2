--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local fromQ = require'Transform'.from_quatp
local toQ = require'Transform'.to_quatp

local t_entry, t_update, t_finish
local timeout = 10.0
local default_plan_timeout = 30

local lco, rco
local okL, qLWaypoint, qLWaistpoint
local okR, qRWaypoint, qRWaistpoint
local quatpL, quatpR
local qWaistDesired
local default_weights = {1,1,0}

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Set where we are
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	local qcWaist = Body.get_waist_command_position()
	local trL = movearm.lPlanner.forward(qcLArm, qcWaist)
	local trR = movearm.rPlanner.forward(qcRArm, qcWaist)

	quatpL = vector.new(toQ(trL))
	quatpR = vector.new(toQ(trR))
	hcm.set_teleop_tflarm(quatpL)
	hcm.set_teleop_tfrarm(quatpR)
	-- TODO: Find the appropriate weights from the position we are in...
	hcm.set_teleop_lweights(default_weights)
	hcm.set_teleop_rweights(default_weights)
	-- Reset the waist
	qWaistDesired = qcWaist
	hcm.set_teleop_waist(qWaistDesired)
	
	local configL = {
		tr = trL, timeout=5,
		via='jacobian_preplan', weights = default_weights
	}
	local configR = {
		tr=trR, timeout=5,
		via='jacobian_preplan', weights = default_weights
	}

	lco, rco, uComp = movearm.goto(configL, configR)
	-- Check for no motion
	okL = type(lco)=='thread' or lco==false
	okR = type(rco)=='thread' or rco==false

end

function state.update()
	--  io.write(state._NAME,' Update\n')
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

	-- Grab the transform
	local quatpL1 = hcm.get_teleop_tflarm()
	local quatpR1 = hcm.get_teleop_tfrarm()
	local qWaistDesired1 = hcm.get_teleop_waist()

	-- Check for changes
	local lChange = quatpL1~=quatpL
	local rChange = quatpR1~=quatpR
	local wChange = qWaistDesired1~=qWaistDesired

	-- Cannot do all. Need an indicator
	if lChange and rChange and wChange then
		print('Too many changes!')
		return
	end

	-- TODO: If *any* change, we should replan all...
	if lChange then
		quatpL = quatpL1
		qWaistDesired = qWaistDesired1
		print(state._NAME, 'L target update')
		local tfL = fromQ(quatpL)
		local via = wChange and 'jacobian_waist_preplan' or 'jacobian_preplan'
		local weights = hcm.get_teleop_lweights()
		local lco1, rco1 = movearm.goto({
			tr = tfL,
			via = via,
			weights = weights,
			qWaistGuess = wChange and qWaistDesired,
			timeout = default_plan_timeout,
		}, false)
		lco = lco1
	end
	if rChange then
		quatpR = quatpR1
		qWaistDesired = qWaistDesired1
		print(state._NAME, 'R target update')
		local tfR = fromQ(quatpR)
		local via = wChange and 'jacobian_waist_preplan' or 'jacobian_preplan'
		local weights = hcm.get_teleop_rweights()
		local lco1, rco1 = movearm.goto(false, {
			tr = tfR,
			via = via,
			weights = weights,
			qWaistGuess = wChange and qWaistDesired,
			timeout = default_plan_timeout,
		})
		rco = rco1
	end
	
	local lStatus = type(lco)=='thread' and coroutine.status(lco)
	local rStatus = type(rco)=='thread' and coroutine.status(rco)

	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()
	local qWaist = Body.get_waist_position()
	if lStatus=='suspended' then
		okL, qLWaypoint, qLWaistpoint = coroutine.resume(lco, qLArm, qWaist)
	end
	if rStatus=='suspended' then
		okR, qRWaypoint, qRWaistpoint = coroutine.resume(rco, qRArm, qWaist)
	end

	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint)
		print(state._NAME, 'R', okR, qRWaypoint)
		-- Safety
		Body.set_larm_command_position(qLArm)
		Body.set_rarm_command_position(qRArm)
		hcm.set_teleop_larm(qLArm)
		hcm.set_teleop_rarm(qRArm)
		return'teleopraw'
	end

	if type(qLWaypoint)=='table' then
		Body.set_larm_command_position(qLWaypoint)
	end
	if type(qRWaypoint)=='table' then
		Body.set_rarm_command_position(qRWaypoint)
	end
	-- Add the waist movement ability
	if qLWaistpoint and qRWaistpoint then
		print('Conflicting waists')
	elseif type(qLWaistpoint)=='table' then
		Body.set_waist_command_position(qLWaistpoint)
	elseif type(qRWaistpoint)=='table' then
		Body.set_waist_command_position(qRWaistpoint)
	end

	-- Check if done
	if lStatus=='dead' and rStatus=='dead' then
		return 'done'
	end

end

function state.exit()
	print(state._NAME..' Exit')
end

return state
