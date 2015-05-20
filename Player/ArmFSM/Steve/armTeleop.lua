--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local fromQ = require'Transform'.from_quatp
local toQ = require'Transform'.to_quatp

local t_entry, t_update, t_finish
local timeout = 10.0

local lco, rco
local okL, qLWaypoint
local okR, qRWaypoint
local quatpL, quatpR
local default_weights = {1,1,0}

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	-- Set where we are
	local qcLArm = Body.get_larm_command_position()
	local qcRArm = Body.get_rarm_command_position()
	local qcWaist = Body.get_waist_command_position()
	local trL = movearm.lPlanner.forward(qcLArm, qcWaist)
	local trR = movearm.rPlanner.forward(qcRArm, qcWaist)

	quatpL = vector.new(toQ(trL))
	quatpR = vector.new(toQ(trR))
	hcm.set_teleop_tflarm(quatpL)
	hcm.set_teleop_tfrarm(quatpR)
	-- TODO: Find the appropriate weights from the position we are in...
	hcm.set_teleop_lweights(default_weights)
	hcm.set_teleop_rweights(default_weights)
	
	local configL = {
		tr = trL, timeout=5,
		via='jacobian_preplan', weights = default_weights
	}
	local configR = {
		tr=trR, timeout=5,
		via='jacobian_preplan', weights = default_weights
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
		print(state._NAME, 'L target update')
		local tfL = fromQ(quatpL1)
		--print(tfL)
		local lco1, rco1 = movearm.goto({
			tr = tfL, timeout = 30, via='jacobian_preplan'
		}, false)
		lco = lco1
		quatpL = quatpL1
	end
	if quatpR1~=quatpR then
		print(state._NAME, 'R target update')
		local tfR = fromQ(quatpR1)
		--print(tfR)
		local lco1, rco1 = movearm.goto(false, {
			tr = tfR, timeout = 30, via='jacobian_preplan'
		})
		rco = rco1
		quatpR = quatpR1
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
