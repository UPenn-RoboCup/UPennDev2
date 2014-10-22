local state = {}
state._NAME = ...
require'hcm'
local Body = require'Body'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 10.0

local T = require'libTransform'
local trRGoal = T.transform6D({0.46,-0.25, 0.15, 0, 0*DEG_TO_RAD, 45*DEG_TO_RAD})
local trLGoal = T.trans(0.46, 0.25, 0.15)
--local qLGoal = vector.zeros(#Body.get_larm_position())

local lPathIter, rPathIter

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
	--
	lPathIter, rPathIter = movearm.goto_tr(trLGoal, trRGoal)
	--lPathIter, rPathIter = movearm.goto_tr(nil, )
	--lPathIter = movearm.goto_q(qLGoal)
end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end
	-- Check the current for collisions
	--print('L Current', Body.get_larm_current()*1)
	--print('R Current', Body.get_rarm_current()*1)
	-- Plan the next joint position
	
	--local qLArm = Body.get_larm_command_position()
	local qLArm = Body.get_larm_position()
	local moreL, q_lWaypoint = lPathIter(qLArm)
	Body.set_larm_command_position(q_lWaypoint)
	
	--local qRArm = Body.get_rarm_command_position()
	local qRArm = Body.get_rarm_position()
	local moreR, q_rWaypoint = rPathIter(qRArm)
	
	local diff, mod_diff
	--print('qRArm', qRArm)
	--print('q_rWaypoint',q_rWaypoint)
	-- Thanks SJ - this *seems* to work
	for i, v in ipairs(qRArm) do
		diff = q_rWaypoint[i] - v
		mod_diff = util.mod_angle(diff)
		--print(diff, mod_diff)
		if math.abs(diff)>math.abs(mod_diff) then q_rWaypoint[i] = v + mod_diff end
	end
	--print()
	
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
