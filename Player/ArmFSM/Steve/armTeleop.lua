--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local fromQ = require'Transform'.from_quaternion
local toQ = require'Transform'.to_quaternion

local t_entry, t_update, t_finish
local timeout = 10.0

local lco, rco
local okL, qLWaypoint
local okR, qRWaypoint
local quatpL, quatpR

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Set where we are
	local qLArm = Body.get_larm_command_position()
	local qRArm = Body.get_rarm_command_position()
	local qWaist = Body.get_waist_command_position()
	quatpL = toQ(movearm.lPlanner.forward(qLArm, qWaist))
	quatpR = toQ(movearm.rPlanner.forward(qLArm, qWaist))
	hcm.set_teleop_tflarm(quatpL)
	hcm.set_teleop_tfrarm(quatpR)
	
	local configL = {
		q=qLArm, timeout=5,
		via='joint_preplan', weights = {0,1,0}
	}
	local configR = {
		tr=qRArm, timeout=5,
		via='joint_preplan', weights = {0,1,0}
	}

	lco, rco, uComp = movearm.goto(configL, configR)
	-- Check for no motion
	okL = lco==false
	okR = rco==false

end

function state.update()
	--  io.write(state._NAME,' Update\n')
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

	-- Grab the transform
	local quatpL1 = hcm.get_teleop_tflarm()
	local quatpR1 = hcm.get_teleop_tfrarm()
	if quatpL1~=quatpL then
		print(state._NAME,'L target')
		local tfL = fromQ({unpack(qL, 1, 4)}, {unpack(qL, 5, 7)})
		print(tfL)
	end
	if quatpR1~=quatpR then
		print(state._NAME,'R target')
		local tfR = fromQ({unpack(qR, 1, 4)}, {unpack(qR, 5, 7)})
		print(tfR)
	end
	
	local lStatus = type(lco)=='thread' and coroutine.status(lco)
	local rStatus = type(rco)=='thread' and coroutine.status(rco)

	local qLArm = Body.get_larm_position()
	local qRArm = Body.get_rarm_position()
	if lStatus=='suspended' then
		okL, qLWaypoint = coroutine.resume(lco, qLArm)
	end
	if rStatus=='suspended' then
		okR, qRWaypoint = coroutine.resume(rco, qRArm)
	end

	if not okL or not okR then
		print(state._NAME, 'L', okL, qLWaypoint)
		print(state._NAME, 'R', okR, qRWaypoint)
		-- Safety
		Body.set_larm_command_position(qLArm)
		Body.set_rarm_command_position(qRArm)
		hcm.set_teleop_larm(qLArm)
		hcm.set_teleop_rarm(qRArm)
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
