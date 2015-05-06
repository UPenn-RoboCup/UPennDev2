local state = {}
state._NAME = ...
local Body = require'Body'
local K = require'K_ffi'
local movearm = require'movearm'

local t_entry, t_update, t_finish
local timeout = 30.0
local lPathIter, rPathIter
local qLGoalFiltered, qRGoalFiltered
local qLD, qRD
local uTorso0, uTorsoComp
local piterators

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

	local fkL = K.forward_larm(teleopLArm)
	local fkR = K.forward_rarm(teleopRArm)

	piterators = movearm.path_iterators({
		{fkL, fkR, 'goto_tr_via_q'}
	})

end

function state.update()
	--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	if not lPathIter or not rPathIter then
		if coroutine.status(piterators)=='dead' then return'done' end
		status, msg = coroutine.resume(piterators)
		-- Escape into raw teleop if no plan in teleop
		if not status then return'teleopraw' end
		if not msg then return'done' end
		-- We are done if the coroutine emits nothing
		lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD, uTorsoComp, uTorso0 = unpack(msg)
		uTorso0 = mcm.get_stance_uTorsoComp()
		uTorso0[3] = 0
		-- Static means true
		lPathIter = lPathIter or true
		rPathIter = rPathIter or true
	end

	-- Timing necessary for next waypoint
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	local moreR, q_rWaypoint = rPathIter(qcRArm, dt)
	local qLNext = moreL and q_lWaypoint or qLGoalFiltered
	local qRNext = moreR and q_rWaypoint or qRGoalFiltered

	local phaseL = moreL and moreL/qLD or 0
	local phaseR = moreR and moreR/qRD or 0
	local phase = math.max(phaseL, phaseR)
	local uTorsoNow = util.se2_interpolate(phase, uTorsoComp, uTorso0)

	-- Send to the joints
	mcm.set_stance_uTorsoComp(uTorsoNow)
	Body.set_larm_command_position(qLNext)
	Body.set_rarm_command_position(qRNext)
  
	if not moreL and not moreR then lPathIter, rPathIter = nil, nil end
end

function state.exit()  
  io.write(state._NAME, ' Exit\n' )
	if not status then
		print(state._NAME, coroutine.status(piterators), status, msg)
	end
end

return state
