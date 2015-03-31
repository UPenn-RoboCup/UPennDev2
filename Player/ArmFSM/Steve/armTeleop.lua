local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'

-- Compensation
-- 1: Use the compensation, but search for the shoulder
-- 2: Use the compenstation, and use the teleop shoulder options
local USE_COMPENSATION = 1

local t_entry, t_update, t_finish
local timeout = 10.0
local lPathIter, rPathIter
local qLGoal, qRGoal
local qLD, qRD
local uTorso0, uTorsoComp
local loptions, roptions

-- Sets uTorsoComp, uTorso0 externall
local function set_iterators(teleopLArm, teleopRArm, teleopComp)
	if teleopComp == 0 then
		-- #nofilter
		uTorsoComp, uTorso0 = nil
		return movearm.goto_q(teleopLArm, teleopRArm, true)
	end
	-- Grab the torso compensation
	local uTorsoAdapt, uTorso = movearm.get_compensation()
	print(uTorsoAdapt, uTorso)
	local uTorsoCompNow = mcm.get_stance_uTorsoComp()
	print(uTorsoCompNow)
	local fkLComp, fkRComp
	fkLComp, fkRComp, uTorsoComp, uTorso0 =
		movearm.apply_q_compensation(teleopLArm, teleopRArm, uTorsoAdapt, uTorso)

	-- Do we have desired null space options?
	if teleopComp == 2 then
		assert('NOT TESTED YET')
		local loptions = hcm.get_teleop_loptions()
		local roptions = hcm.get_teleop_roptions()
		-- Form the iterator
		--return movearm.goto_tr_via_q(fkLComp, fkRComp, loptions, roptions)
		return movearm.goto_tr_stack(fkLComp, fkRComp, loptions, roptions)
		--return movearm.goto_tr(fkLComp, fkRComp, loptions, roptions)
	else
		-- Form the iterator
		--return movearm.goto_tr_via_q(fkLComp, fkRComp)
		return movearm.goto_tr_stack(fkLComp, fkRComp)
		--return movearm.goto_tr(fkLComp, fkRComp)
	end
end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
	local qcLArm0 = Body.get_larm_command_position()
	local qcRArm0 = Body.get_rarm_command_position()
	local teleopLArm = hcm.get_teleop_larm()
	local teleopRArm = hcm.get_teleop_rarm()
	local teleopComp = hcm.get_teleop_compensation()

	lPathIter, rPathIter, qLGoal, qRGoal, qLD, qRD = set_iterators(teleopLArm, teleopRArm, teleopComp)

	--loptions = {qLGoal[3], 0}
	--roptions = {qRGoal[3], 0}
	--hcm.set_teleop_loptions(loptions or {qcLArm[3], 0})
	--hcm.set_teleop_roptions(roptions or {qcRArm[3], 0})

end

function state.update()
	--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

	-- Update our measurements availabl in the state
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()

	-- Timing necessary
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	local moreR, q_rWaypoint = rPathIter(qcRArm, dt)

	local qLNext = moreL and q_lWaypoint or qLGoal
	local qRNext = moreR and q_rWaypoint or qRGoal

	local phaseL = moreL and moreL/qLD or 0
	local phaseR = moreR and moreR/qRD or 0

	assert(uTorsoComp)
	assert(uTorso0)
	local phase = math.max(phaseL, phaseR)
	local uTorsoNow = util.se2_interpolate(phase, uTorsoComp, uTorso0)
	mcm.set_stance_uTorsoComp(uTorsoNow)

	-- Send to the joints
	Body.set_larm_command_position(qLNext)
	Body.set_rarm_command_position(qRNext)
  
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
