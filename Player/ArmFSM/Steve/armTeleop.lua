--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local movearm = require'movearm'

local t_entry, t_update, t_finish
local timeout = 30.0

local lco, rco, uComp
local okL, qLWaypoint
local okR, qRWaypoint

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	local configL = {
		q=hcm.get_teleop_larm(), t=10,
		via='jacobian', weights = {0,1,0}
	}
	local configR = {
		q=hcm.get_teleop_rarm(), t=10,
		via='jacobian', weights = {0,1,0}
	}

	-- Always add compensation
	lco, rco, uComp = movearm.goto(configL, configR, true)
	okL = false
	okR = false

end

function state.update()
	--  io.write(state._NAME,' Update\n')
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	local lStatus = type(lco)=='thread' and coroutine.status(lco)
	local rStatus = type(rco)=='thread' and coroutine.status(rco)

	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()
	if lStatus=='suspended' then okL, qLWaypoint = coroutine.resume(lco, qLArm) end
	if rStatus=='suspended' then okR, qRWaypoint = coroutine.resume(rco, qRArm) end

	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint)
		print(state._NAME, 'R', okR, qRWaypoint)
		local qcLArm = Body.get_larm_command_position()
		local qcRArm = Body.get_rarm_command_position()
		hcm.set_teleop_larm(qcLArm)
		hcm.set_teleop_rarm(qcRArm)
		return'teleopraw'
	end

	if type(qLWaypoint)=='table' then
		Body.set_larm_command_position(qLWaypoint)
	end
	if type(qRWaypoint)=='table' then
		Body.set_rarm_command_position(qRWaypoint)
	end

	-- Check if done
	if lStatus=='dead' and rStatus=='dead' then
		return 'done'
	end

end

function state.exit()
	print(state._NAME..' Exit')
end

return state
