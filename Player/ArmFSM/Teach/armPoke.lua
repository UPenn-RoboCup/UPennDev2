local state = {}
state._NAME = ...
require'hcm'
local Body = require'Body'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 20.0

local T = require'libTransform'
local K = require'K_ffi'

local lPathIter, rPathIter
local grip0, is_open

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
	
	local qR = Body.get_rarm_position()
	local fR = K.forward_r_arm(qR)
	local trRGoal = T.trans(0.1,0,0) * fR
	--local trRGoal = T.trans(0,0.1,0) * fR
	--
	local qL = Body.get_larm_position()
	local fL = K.forward_l_arm(qL)
	local trLGoal = fL
	--
	lPathIter, rPathIter = movearm.goto_tr(trLGoal, trRGoal, {20*DEG_TO_RAD}, {-75*DEG_TO_RAD})
	-- Let the trigger detect impact, so open it
	Body.set_rgrip_command_position(45*DEG_TO_RAD)
	is_open = false
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
	
	-- Wait until the gripper is open before moving
	if not is_open then
		grip0 = Body.get_rgrip_position()
		if vector.norm(grip0 - Body.get_rgrip_command_position()) > 2*DEG_TO_RAD then
			return
		else
			print('GRIPPER OPENED!')
			is_open = true
			Body.set_rgrip_torque_enable(0)
		end
	end
	
	-- Check if we have touched anything
	local grip = Body.get_rgrip_position()
	local grip_diff = grip - grip0
	if grip_diff[1] > 2*DEG_TO_RAD or grip_diff[2] > 2*DEG_TO_RAD then
		print('grip_diff', grip_diff)
		return 'touch'
	end
	
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
	
	--[[
	local qRArm = Body.get_rarm_command_position()
	local moreR, q_rWaypoint = rPathIter(qRArm, dt)
	--]]
	----[[
	local qRArm = Body.get_rarm_position()
	local moreR, q_rWaypoint = rPathIter(qRArm)
	--]]
	Body.set_rarm_command_position(q_rWaypoint)
	-- Check if done
	if not moreL and not moreR then
		print('DONE POKE')
		return 'done'
	end
end

function state.exit()  
  print(state._NAME..' Exit' )
	-- Re-enable the torque on the gripper
	Body.set_rgrip_torque_enable(1)
end

return state
