local state = {}
state._NAME = ...
local Body = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish, t_command
local timeout = 30.0

local piterators
local lPathIter, rPathIter
local qLD, qRD
local uTorsoComp, uTorso0
local qLGoalFiltered, qRGoalFiltered

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- TODO: Autodetect which stges to use, based on our initial position
	piterators = movearm.path_iterators(Config.arm.readyFromInitStages)

end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	if not lPathIter or not rPathIter then
		local it
		it, uTorsoComp, uTorso0 = piterators()
		-- We are done if the coroutine emits nothing
		if not it then return'done' end
		lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD = unpack(it)
		hcm.set_teleop_loptions({qLGoalFiltered[3], 0})
		hcm.set_teleop_roptions({qRGoalFiltered[3], 0})
	end

	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	--local qLArm = Body.get_larm_position()
	--local qRArm = Body.get_rarm_position()

	-- Find the next arm position
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	local moreR, q_rWaypoint = rPathIter(qcRArm, dt)
	local qLNext = moreL and q_lWaypoint or qLGoalFiltered
	local qRNext = moreR and q_rWaypoint or qRGoalFiltered
	--print(moreL, q_lWaypoint, qLGoalFiltered)

	-- Find the torso compensation position
	local phaseL = moreL and moreL/qLD or 0
	local phaseR = moreR and moreR/qRD or 0
	local phase = math.max(phaseL, phaseR)
	local uTorsoNow = util.se2_interpolate(phase, uTorsoComp, uTorso0)

	-- Set the arm and torso commands
	mcm.set_stance_uTorsoComp(uTorsoNow)
	Body.set_larm_command_position(qLNext)
	Body.set_rarm_command_position(qRNext)
	t_command = t

	-- Check if done and reset the iterators
	if not moreL and not moreR then lPathIter, rPathIter = nil, nil end
end

function state.exit()
  print(state._NAME..' Exit' )
	-- For teleop if called next
	--hcm.set_teleop_compensation(2)
end

return state
