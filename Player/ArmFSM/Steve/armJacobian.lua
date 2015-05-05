local state = {}
state._NAME = ...
local Body = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local T = require'Transform'
local t_entry, t_update, t_finish, t_command
local timeout = 30.0

local piterators, status, msg
local lPathIter, rPathIter
local qLD, qRD
local uTorso0, uTorsoComp
local qLGoalFiltered, qRGoalFiltered

local trL = T.transform6D{0.35, 0.25, 0.2, 0, 0, -45*DEG_TO_RAD}
local trR = T.transform6D{0.35, -0.25, 0.2, 0, 0, 45*DEG_TO_RAD}

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry


	lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered =
		movearm.goto_jacobian_stack(trL, trR)

	print('Entry',lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered)

end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	--local qLArm = Body.get_larm_position()
	--local qRArm = Body.get_rarm_position()

	-- Find the next arm position
	local moreL, q_lWaypoint = lPathIter(qcLArm, dt)
	local moreR, q_rWaypoint = rPathIter(qcRArm, dt)
	local qLNext = moreL and q_lWaypoint or qLGoalFiltered
	local qRNext = moreR and q_rWaypoint or qRGoalFiltered

	Body.set_larm_command_position(qLNext)
	Body.set_rarm_command_position(qRNext)
	t_command = t

	-- Check if done and reset the iterators
	if not moreL and not moreR then
		lPathIter, rPathIter = nil, nil
		return'done'
	end
end

function state.exit()
  print(state._NAME..' Exit' )
	if not status then print(status, msg) end
	-- For teleop if called next
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	hcm.set_teleop_larm(qcLArm)
  hcm.set_teleop_rarm(qcRArm)
end

return state
