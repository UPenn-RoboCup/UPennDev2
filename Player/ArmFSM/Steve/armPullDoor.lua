--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local plugins = require'armplugins'

local t_entry, t_update, t_finish
local timeout = 30.0

local pco, lco, rco

local pStatus, lmovement, rmovement

function state.entry()
  io.write(state._NAME, ' Entry\n')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	local model ={
		x = 0.52,
		y = -0.19,
		z = -0.05,
		yaw = 0,
		hinge = -1,
		roll = -math.pi/2,
		hand = 'right'
	}
	pco, lco, rco = plugins.gen('pulldoor', model)
	okL = false
	okR = false

end

function state.update()
	--  io.write(state._NAME,' Update\n')
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

	-- Evaluate the model
	local pStatus = type(pco)=='thread' and coroutine.status(pco)
	if not pStatus then
		-- There may be some error, since pco is not a thread
		print('pco | Failed to start')
		return'teleopraw'
	elseif pStatus=='dead' then
		return 'done'
	elseif pStatus=='suspended' then
		okP, lmovement, rmovement = coroutine.resume(pco)
		-- Check for errors
		if not okP then
			print(state._NAME, 'pco', okL, lmovement)
			return'teleopraw'
		elseif lmovement.via then
			-- A new movement type for lco
		end
	end

	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()

	local lStatus = type(lco)=='thread' and coroutine.status(lco)
	local rStatus = type(rco)=='thread' and coroutine.status(rco)

	if not lStatus then
		print('lco | Failed to start')
		return'teleopraw'
	elseif lStatus=='suspended' then
		local okL, qLWaypoint =
			coroutine.resume(lco, qLArm, unpack(lmovement))
		if okL and type(qLWaypoint)=='table'then
			Body.set_larm_command_position(qLWaypoint)
		else
			print(state._NAME, 'lco', okL, qLWaypoint)
			return'teleopraw'
		end
	end
	if not rStatus then
		print('rco | Failed to start')
		return'teleopraw'
	elseif rStatus=='suspended' then
		local okR, qRWaypoint =
			coroutine.resume(rco, qRArm, unpack(rmovement))
		if okR and type(qRWaypoint)=='table' then
			Body.set_rarm_command_position(qRWaypoint)
		else
			print(state._NAME, 'rco', okR, qRWaypoint)
			return'teleopraw'
		end
	end

	-- Check if done
	if lStatus=='dead' and rStatus=='dead' then
		-- Should this be possible with pco? Maybe, should just regen
		return 'done'
	end

	-- Set the compensation: Not needed
	--mcm.set_stance_uTorsoComp(uComp)

end

function state.exit()
	io.write(state._NAME, ' Exit\n')

	local qcLArm = Body.get_larm_position()
	local qcRArm = Body.get_rarm_position()
	hcm.set_teleop_larm(qcLArm)
  hcm.set_teleop_rarm(qcRArm)
end

return state
