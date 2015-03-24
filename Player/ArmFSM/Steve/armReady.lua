local state = {}
state._NAME = ...
local Body = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 10.0

local T = require'Transform'

local trLGoal = T.transform6D(Config.arm.trLArm1)
local trRGoal = T.transform6D(Config.arm.trRArm1)

local lPathIter, rPathIter
local qLD, qRD
local qLGoalFiltered, qRGoalFiltered

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry


	-- Grab the torso compensation
	local uTorsoAdapt, uTorso = movearm.get_compensation()
	uTorso0 = uTorso
	uTorsoComp = util.pose_relative(uTorsoAdapt, uTorso0)
	-- Apply the compensation
	local trComp = T.trans(-uTorsoComp[1],-uTorsoComp[2], 0)
	local trLArmComp = trComp * trLGoal
	local trRArmComp = trComp * trRGoal
	-- Form the iterator
	lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD =
		movearm.goto_tr(trLArmComp, trRArmComp)
	--[[
	lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD =
		movearm.goto_tr_via_q(fkLComp, fkRComp)
	--]]

	-- no compensation
	--lPathIter, rPathIter = movearm.goto_tr(trLGoal, trRGoal)
  --lPathIter, rPathIter = movearm.goto_tr_via_q(trLGoal, trRGoal)
end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end
	-- Timing necessary

	local qLArm = Body.get_larm_command_position()
	local moreL, q_lWaypoint = lPathIter(qLArm, dt)

	local qRArm = Body.get_rarm_command_position()
	local moreR, q_rWaypoint = rPathIter(qRArm, dt)

	local phaseL = moreL and moreL/qLD or 0
	local phaseR = moreR and moreR/qRD or 0
	local phase = math.max(phaseL, phaseR)
	local uTorsoNow = util.se2_interpolate(phase, uTorsoComp, uTorso0)
	mcm.set_stance_uTorsoComp(uTorsoNow)

	Body.set_larm_command_position(q_lWaypoint)
	Body.set_rarm_command_position(q_rWaypoint)

	-- Check if done
	if not moreL and not moreR then return 'done' end
end

function state.exit()
  print(state._NAME..' Exit' )
	-- For teleop if called next
	hcm.set_teleop_compensation(2)
	hcm.set_teleop_loptions({qLGoalFiltered[3], 0})
	hcm.set_teleop_roptions({qRGoalFiltered[3], 0})
end

return state
