local state = {}
state._NAME = ...
require'hcm'
local Body = require'Body'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 20.0

local T = require'Transform'
local K = require'K_ffi'

local lPathIter, rPathIter
local grip0, is_open
local grip_open = vector.new{-100, -90}*DEG_TO_RAD

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
	
	local qR = Body.get_rarm_position()
	local fR, paramR = K.forward_rarm(qR)
	local trRGoal = fR * T.trans(0.10,0,0)
	--local trRGoal = fR * T.trans(0,0.1,0)
	--
	local qL = Body.get_larm_position()
	local fL, paramL = K.forward_larm(qL)
	local trLGoal = fL
	--
	lPathIter, rPathIter = movearm.goto_tr(trLGoal, trRGoal, paramL, paramR)
	-- Let the trigger detect impact, so open it
	--
	is_open = false
	for k=1,3 do
		print('SETUP!')
		Body.set_rgrip_torque_enable(1)
		Body.set_rgrip_mode('position')
		if not WEBOTS then unix.usleep(1e4) end
	end
		
end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then print('POKE TIMEOUT'); return'timeout' end
	
	-- Wait until the gripper is open before moving
	if not is_open then
		Body.set_rgrip_command_position(grip_open)
		grip0, tgrip0 = Body.get_rgrip_position()
		local grip_diff = grip0 - grip_open
		--print('GRIP', grip_diff*RAD_TO_DEG)
		--print('T', tgrip0 - t_entry*vector.ones(#tgrip0))
		if (math.abs(grip_diff[1]) < 2*DEG_TO_RAD and tgrip0[1]-t_entry>0.5) and (math.abs(grip_diff[2]) < 2*DEG_TO_RAD and tgrip0[2]-t_entry>0.5) then
			for k=1,3 do
				Body.set_rgrip_torque_enable(0)
				if not WEBOTS then unix.usleep(1e4) end
			end
			print('GRIPPER OPENED!')
			is_open = true
		end
		return
	end
	
	-- Check if we have touched anything
	local grip = Body.get_rgrip_position()
	local grip_diff = grip - grip0
	local touchTrigger = math.abs(grip_diff[1]) > 1*DEG_TO_RAD
	local touchVice = math.abs(grip_diff[2]) > 1*DEG_TO_RAD
	if touchTrigger or touchVice then
		print('Trigger', touchTrigger)
		print('Vice', touchVice)
		print('grip_diff', grip_diff*RAD_TO_DEG, grip0)
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
		print('NO POKE TOUCH')
		return 'done'
	end
end

function state.exit()  
  print(state._NAME..' Exit' )
	-- Re-enable the torque on the gripper
	Body.set_rgrip_torque_enable(1)
end

return state
