--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local plugins = require'armplugins'

local t_entry, t_update, t_finish
local timeout = 30.0

local USE_COMPENSATION = false
local lco, rco, uComp
local okL, qLWaypoint
local okR, qRWaypoint

local dir = 1
local USE_PLUGIN = true
local pco

local pStatus, vwDoor, weightsDoor, qArmGuessDoor

function state.entry()
  io.write(state._NAME, ' Entry\n')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	local configL = {
		vw = {0,-1*dir,0, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		via='jacobian_velocity',
		weights = {1,0,0},
		timeout=300
	}
	local configR = {
		vw = {0,1*dir,0, 0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD},
		via='jacobian_velocity',
		weights = {1,0,0},
		timeout=3e5
	}
	dir = -dir

	if USE_PLUGIN then
		local model ={
			x = 0.52,
			y = -0.19,
			z = -0.05,
			yaw = 0,
			hinge = -1,
			roll = -math.pi/2,
			hand = 'right'

		}
		if not pco then
			pco = plugins.gen('pulldoor', model)
		end
	end

	lco, rco, uComp = movearm.goto(configL, configR, USE_COMPENSATION)
	okL = false
	okR = false
	if uComp then print('uComp', unpack(uComp)) end

end

function state.update()
	--  io.write(state._NAME,' Update\n')
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

	local lStatus = type(lco)=='thread' and coroutine.status(lco)
	local rStatus = type(rco)=='thread' and coroutine.status(rco)

	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()
	if lStatus=='suspended' then
		okL, qLWaypoint = coroutine.resume(lco, qLArm)
	end
	if rStatus=='suspended' then
		okR, qRWaypoint = coroutine.resume(rco, qRArm, vwDoor, weightsDoor, qArmGuessDoor)
	end

	-- Try the model
	if coroutine.status(pco)=='suspended' then
		pStatus, vwDoor, weightsDoor, qArmGuessDoor = coroutine.resume(pco, qLArm, qRArm)
		if not pStatus then
			print('pco', pStatus, vwDoor)
			vwDoor = false
		end
	end

	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint)
		print(state._NAME, 'R', okR, qRWaypoint)
		return'teleopraw'
	end

	if type(qLWaypoint)=='table' then
		--Body.set_larm_command_position(qLWaypoint)
	end
	if type(qRWaypoint)=='table' then
		Body.set_rarm_command_position(qRWaypoint)
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
