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

local lco, rco
local okL, qLWaypoint
local okR, qRWaypoint

function state.entry()
  io.write(state._NAME, ' Entry\n')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	local configL = {
		q=hcm.get_teleop_larm(), t=5,
		via='jacobian', weights = {1,0,0}
	}
	local configR = {
		q=hcm.get_teleop_rarm(), t=5,
		via='jacobian', weights = {1,0,0}
	}

	lco, rco = movearm.goto(configL, configR)
	okL = false
	okR = false

end

function state.update()
	--  io.write(state._NAME,' Update\n')
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	local lStatus, rStatus
	if type(lco)=='thread' then
		lStatus = coroutine.status(lco)
	else
		print('L', lco)
	end
	if type(rco)=='thread' then
		rStatus = coroutine.status(rco)
	else
		print('R', rco)
	end

	--local qcLArm = Body.get_larm_command_position()
	--local qcRArm = Body.get_rarm_command_position()

	if lStatus=='suspended' then okL, qLWaypoint = coroutine.resume(lco) end
	if rStatus=='suspended' then okR, qRWaypoint = coroutine.resume(rco) end
	if not okL and not okR then return'escape' end

	if type(qLWaypoint)=='table' then
		Body.set_larm_command_position(qLWaypoint)
	end
	if type(qRWaypoint)=='table' then
		Body.set_rarm_command_position(qRWaypoint)
	end

	-- Catch the errors
	if not okL then
		print(okL, qLWaypoint)
	end
	if not okR then
		print(okR, qRWaypoint)
	end

	-- Check if done
	if lStatus=='dead' and rStatus=='dead' then
		return 'done'
	end

end

function state.exit()
	io.write(state._NAME, ' Exit\n')

	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	hcm.set_teleop_larm(qcLArm)
  hcm.set_teleop_rarm(qcRArm)
end

return state
