local state = {}
state._NAME = ...
local Body = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish, t_command
local timeout = 30.0

local piterators, status, msg
local lPathIter, rPathIter
local qLD, qRD
local uTorso0, uTorsoComp
local qLGoalFiltered, qRGoalFiltered

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- TODO: Autodetect which stges to use, based on our initial position
	piterators = movearm.path_iterators(Config.arm.readyFromInitStages)

	status, msg = nil, nil

end

function state.update()
  --io.write(state._NAME, ' Update\n' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	if not lPathIter or not rPathIter then
		if coroutine.status(piterators)=='dead' then return'done' end
		status, msg = coroutine.resume(piterators)
		if not status or not msg then return'done' end
		-- We are done if the coroutine emits nothing
		lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD, uTorsoComp, uTorso0 = unpack(msg)
		uTorso0 = mcm.get_stance_uTorsoComp()
		uTorso0[3] = 0
		-- Static means true
		lPathIter = lPathIter or true
		rPathIter = rPathIter or true
	end

	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()

	-- Find the next arm position
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	local moreR, q_rWaypoint = rPathIter(qcRArm, dt)
	local qLNext = moreL and q_lWaypoint or qLGoalFiltered
	local qRNext = moreR and q_rWaypoint or qRGoalFiltered

	-- Find the torso compensation position
	if uTorsoComp then
		--[[
		print(state._NAME..' | uTorso0', uTorso0)
		print(state._NAME..' | uTorsoComp', uTorsoComp)
		print(state._NAME..' | uTorsoNow', uTorsoNow)
		--]]
		local phaseL = moreL and moreL/qLD or 0
		local phaseR = moreR and moreR/qRD or 0
		local phase = math.max(phaseL, phaseR)
		local uTorsoNow = util.se2_interpolate(phase, uTorsoComp, uTorso0)
		-- Set the arm and torso commands
		mcm.set_stance_uTorsoComp(uTorsoNow)
	end
	Body.set_larm_command_position(qLNext)
	Body.set_rarm_command_position(qRNext)
	t_command = t

	-- Check if done and reset the iterators
	if not moreL and not moreR then lPathIter, rPathIter = nil, nil end
end

function state.exit()
  io.write(state._NAME, ' Exit\n' )
	if not status then print(status, msg) end
	-- For teleop if called next
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	hcm.set_teleop_larm(qcLArm)
  hcm.set_teleop_rarm(qcRArm)
end

return state
