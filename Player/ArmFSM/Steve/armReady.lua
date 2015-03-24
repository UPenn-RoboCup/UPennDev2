local state = {}
state._NAME = ...
local Body = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 10.0

local T = require'Transform'

local stage
-- Stages: L, R (Transforms/joints), type of motion
local stages = {
	[1] = {T.transform6D(Config.arm.trLArm1), T.transform6D(Config.arm.trRArm1), 'goto_tr',},
	[2] = {T.transform6D(Config.arm.trLArm2), T.transform6D(Config.arm.trRArm2), 'goto_tr',}
}

local lPathIter, rPathIter
local qLD, qRD
local qLGoalFiltered, qRGoalFiltered

-- TODO: Allow no compensation
local function get_iters(s)
	-- Grab the goals and the method
	local lGoal, rGoal, via = unpack(stages[s])
	-- NOTE: Compensation requires goto_tr or goto_tr_via_q...
	local go = movearm[via]

	-- FK goals for use with copmensation
	local fkLComp, fkRComp
	if via:find'tr' then
		fkLComp, fkRComp, uTorsoComp, uTorso0 =
			movearm.apply_tr_compensation(lGoal, rGoal, movearm.get_compensation())
	else
		fkLComp, fkRComp, uTorsoComp, uTorso0 =
			movearm.apply_q_compensation(lGoal, rGoal, movearm.get_compensation())
	end

	-- Form the iterator
	return go(fkLComp, fkRComp)
end

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- TODO: Autodetect a stage, based on our current position
	stage = 1
end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	if not lPathIter or not rPathIter then
		lPathIter, rPathIter, qLGoalFiltered, qRGoalFiltered, qLD, qRD = get_iters(stage)
	end

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
	if not moreL and not moreR then
		-- Reset the iterators
		lPathIter, rPathIter = nil, nil
		if stage==2 then
			return 'done'
		else
			stage = stage + 1
		end
	end
end

function state.exit()
  print(state._NAME..' Exit' )
	-- For teleop if called next
	hcm.set_teleop_compensation(2)
	hcm.set_teleop_loptions({qLGoalFiltered[3], 0})
	hcm.set_teleop_roptions({qRGoalFiltered[3], 0})
end

return state
