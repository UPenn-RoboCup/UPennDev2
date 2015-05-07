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

local USE_COMPENSATION = true
local lco, rco, uComp
local okL, qLWaypoint
local okR, qRWaypoint

function state.entry()
  io.write(state._NAME, ' Entry\n')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	lco, rco, uComp = movearm.goto(Config.arm.configL1, Config.arm.configR1, USE_COMPENSATION)
	okL = false
	okR = false
	if uComp then print('uComp', unpack(uComp)) end

end

function state.update()
	--  io.write(state._NAME,' Update\n')
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end

	if type(lco)~='thread' then print('L', lco) end
	if type(rco)~='thread' then print('R', lco) end

	local lStatus = coroutine.status(lco)
	local rStatus = coroutine.status(rco)
	--local qcLArm = Body.get_larm_command_position()
	--local qcRArm = Body.get_rarm_command_position()

	if lStatus=='suspended' then okL, qLWaypoint = coroutine.resume(lco) end
	if rStatus=='suspended' then okR, qRWaypoint = coroutine.resume(rco) end
	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint)
		print(state._NAME, 'R', okR, qRWaypoint)
		return'teleopraw'
	end

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

	-- Set the compensation: Not needed
	--mcm.set_stance_uTorsoComp(uComp)

end

function state.exit()
	io.write(state._NAME, ' Exit\n')

	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	hcm.set_teleop_larm(qcLArm)
  hcm.set_teleop_rarm(qcRArm)
end

return state
