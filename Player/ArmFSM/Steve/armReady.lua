local state = {}
state._NAME = ...
local Body = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 10.0

local T = require'Transform'
local trLGoal = T.transform6D{0.2, 0.25, -0.05, 0, 0, -90*DEG_TO_RAD}
local trRGoal = T.transform6D({0.2, -0.25, -0.12, 0, 0, 90*DEG_TO_RAD})

local lPathIter, rPathIter

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
	--lPathIter, rPathIter = movearm.goto_tr(trLGoal, trRGoal, {-20*DEG_TO_RAD}, {10*DEG_TO_RAD})
  lPathIter, rPathIter = movearm.goto_tr_via_q(trLGoal, trRGoal, {-25*DEG_TO_RAD}, {25*DEG_TO_RAD})
end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end
	-- Timing necessary
	----[[
	local qLArm = Body.get_larm_command_position()
	local moreL, q_lWaypoint = lPathIter(qLArm, dt)
	--]]
	-- No time needed
	--[[
	local qLArm = Body.get_larm_position()
	local moreL, q_lWaypoint = lPathIter(qLArm)
	--]]
	Body.set_larm_command_position(q_lWaypoint)

	----[[
	local qRArm = Body.get_rarm_command_position()
	local moreR, q_rWaypoint = rPathIter(qRArm, dt)
	--]]
	--[[
	local qRArm = Body.get_rarm_position()
	local moreR, q_rWaypoint = rPathIter(qRArm)
	--]]
	Body.set_rarm_command_position(q_rWaypoint)
	-- Check if done
	if not moreL and not moreR then
		return 'done'
	end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
