--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local USE_SAFE_YAW = true
local IS_YAW_SAFE

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

	local qL = Body.get_larm_position()
	local qR = Body.get_rarm_position()

	IS_YAW_SAFE = true
  if USE_SAFE_YAW then
    -- Try to avoid self collisions
    qL[3] = -20*DEG_TO_RAD
    qR[3] = 20*DEG_TO_RAD
    lco, rco = movearm.goto({
				q = qL, duration = 5, timeout = 7, via='jointspace'
			},{
				q = qR, duration = 5, timeout = 7, via='jointspace'
			})
    IS_YAW_SAFE = false
	else
		lco, rco = movearm.goto(Config.arm.trLArm0, Config.arm.trRArm0)
  end
	okL = false
	okR = false

	-- Set Hardware limits in case
  for i=1,5 do
    Body.set_larm_torque_enable(1)
    Body.set_rarm_torque_enable(1)
    Body.set_larm_command_velocity(500)
    Body.set_rarm_command_velocity(500)
    Body.set_larm_command_acceleration(50)
    Body.set_rarm_command_acceleration(50)
    Body.set_larm_position_p(8)
    Body.set_rarm_position_p(8)
    if not IS_WEBOTS then unix.usleep(1e5) end
  end
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

	-- Check if errors in either
	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint, lco)
		print(state._NAME, 'R', okR, qRWaypoint, rco)
		-- Safety
		Body.set_larm_command_position(qLArm)
		Body.set_rarm_command_position(qRArm)
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
		if IS_YAW_SAFE then return 'done' end
		IS_YAW_SAFE = true
		-- No longer need to use for future arm motions
		USE_SAFE_YAW = false
		lco, rco = movearm.goto(Config.arm.trLArm0, Config.arm.trRArm0)
	end

end

function state.exit()
	io.write(state._NAME, ' Exit\n')

	-- Undo the hardware limits
  for i=1,3 do
    Body.set_larm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_rarm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_larm_command_acceleration({200,200,200,200,200,200,200})
    Body.set_rarm_command_acceleration({200,200,200,200,200,200,200})
    Body.set_larm_position_p(32)
    Body.set_rarm_position_p(32)
    if not IS_WEBOTS then unix.usleep(1e5) end
  end

	if not okL or not okR then
		local qLArm = Body.get_larm_position()
		local qRArm = Body.get_rarm_position()
		hcm.set_teleop_larm(qLArm)
		hcm.set_teleop_rarm(qRArm)
	else
		local qcLArm = Body.get_larm_command_position()
		local qcRArm = Body.get_rarm_command_position()
		hcm.set_teleop_larm(qcLArm)
		hcm.set_teleop_rarm(qcRArm)
	end
end

return state
