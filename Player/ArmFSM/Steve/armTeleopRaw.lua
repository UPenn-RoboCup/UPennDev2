local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'

-- Compensation
-- 1: Use the compensation, but search for the shoulder
-- 2: Use the compenstation, and use the teleop shoulder options
local USE_COMPENSATION = 1

local t_entry, t_update, t_finish
local timeout = 30.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qLD, qRD
local okL, okR

local teleopLArm
local teleopRArm

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
	hcm.set_teleop_larm(teleopLArm)
	hcm.set_teleop_rarm(teleopRArm)

	lco, rco = movearm.goto({
		q = teleopLArm, timeout = 5, via='joint_preplan'

	}, {
		q = teleopRArm, timeout = 5, via='joint_preplan'
	})

	okL = type(lco)=='thread'
	okR = type(rco)=='thread'

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

	if teleopLArm~=teleopLArm1 then
		teleopLArm = teleopLArm1
		print(state._NAME,'L target', teleopLArm)
		local lco1, rco1 = movearm.goto({
			q = teleopLArm, timeout = 30, via='joint_preplan'
		}, false)
		lco = lco1
	end
	if teleopRArm~=teleopRArm1 then
		teleopRArm = teleopRArm1
		print(state._NAME,'R target', teleopRArm)
		local lco1, rco1 = movearm.goto(false, {
			q = teleopRArm, timeout = 30, via='joint_preplan'
		})
		rco = rco1
	end

	local lStatus = type(lco)=='thread' and coroutine.status(lco)
	local rStatus = type(rco)=='thread' and coroutine.status(rco)

	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()
	if lStatus=='suspended' then okL, qLWaypoint = coroutine.resume(lco, qLArm) end
	if rStatus=='suspended' then okR, qRWaypoint = coroutine.resume(rco, qRArm) end

	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint)
		print(state._NAME, 'R', okR, qRWaypoint)
		-- Safety
		Body.set_larm_command_position(qLArm)
		Body.set_rarm_command_position(qRArm)
		return'teleopraw'
	end

	if type(qLWaypoint)=='table' then
		Body.set_larm_command_position(qLWaypoint)
	end
	if type(qRWaypoint)=='table' then
		Body.set_rarm_command_position(qRWaypoint)
	end

	-- Check if done
	if lStatus=='dead' and rStatus=='dead' then
		return 'done'
	end

end

function state.exit()
  print(state._NAME..' Exit' )

	if not okL or not okR then
		local qLArm = Body.get_larm_position()
		local qRArm = Body.get_rarm_position()
		hcm.set_teleop_larm(qLArm)
		hcm.set_teleop_rarm(qRArm)
	else
		local qcLArm = Body.get_larm_command_position()
		local qcRArm = Body.get_rarm_command_position()
		hcm.set_teleop_larm(qcLArm)
		hcm.set_teleop_rarm(qcRArm)
	end

end

return state
