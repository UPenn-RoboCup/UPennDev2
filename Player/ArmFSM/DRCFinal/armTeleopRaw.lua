local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'

local t_entry, t_update, t_finish
local timeout = 30.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qLD, qRD

local okL, qLWaypoint, qLWaistpoint
local okR, qRWaypoint, qRWaistpoint
local default_plan_timeout = 30

local teleopLArm, teleopRArm, teleopWaist

function state.entry()
  print(state._NAME..' Entry')
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Set where we are
	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()
	local qWaist = Body.get_waist_position()
	teleopLArm = qLArm
	teleopRArm = qRArm
	teleopWaist = qWaist
	hcm.set_teleop_larm(teleopLArm)
	hcm.set_teleop_rarm(teleopRArm)
	hcm.set_teleop_waist(teleopWaist)

	lco, rco = movearm.goto({
		q = teleopLArm, timeout = 5, via='joint_preplan'
	}, {
		q = teleopRArm, timeout = 5, via='joint_preplan'
	})

	-- Check for no motion
	okL = type(lco)=='thread' or lco==false
	okR = type(rco)=='thread' or rco==false

end

function state.update()
	--io.write(state._NAME, ' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

	local teleopLArm1 = hcm.get_teleop_larm()
	local teleopRArm1 = hcm.get_teleop_rarm()
	local teleopWaist1 = hcm.get_teleop_waist()

	-- Check for changes
	local lChange = teleopLArm1~=teleopLArm
	local rChange = teleopRArm1~=teleopRArm
	local wChange = teleopWaist1~=teleopWaist

	-- Cannot do all. Need an indicator
	if lChange and rChange and wChange then
		print('Too many changes!')
		return
	end

	-- TODO: If *any* change, we should replan all...
	if lChange then
		teleopLArm = teleopLArm1
		teleopWaist = teleopWaist1
		print(state._NAME,'L target', teleopLArm)
		local via = wChange and 'joint_waist_preplan' or 'joint_preplan'
		local lco1, rco1 = movearm.goto({
			q = teleopLArm,
			via = via,
			qWaistGuess = wChange and teleopWaist,
			timeout = default_plan_timeout
		}, false)
		lco = lco1
	end
	if rChange then
		teleopRArm = teleopRArm1
		teleopWaist = teleopWaist1
		print(state._NAME, 'R target', teleopRArm)
		local via = wChange and 'joint_waist_preplan' or 'joint_preplan'
		local lco1, rco1 = movearm.goto(false, {
			q = teleopRArm,
			via = via,
			qWaistGuess = wChange and teleopWaist,
			timeout = default_plan_timeout
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
		Body.set_waist_command_position(qWaist)
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
  print(state._NAME..' Exit' )
end

return state
