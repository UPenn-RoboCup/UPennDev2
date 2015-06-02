local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'
local t_entry, t_update

local HT = require'libHeadTransform'
local T = require'Transform'

-- Neck limits
local headSpeed = 30 * DEG_TO_RAD * vector.ones(2)
local headThresh = 1 * DEG_TO_RAD * vector.ones(2)

local function get_world_tf()
	local bH = mcm.get_stance_bodyHeight()
	local uTorso = mcm.get_stance_uTorsoComp()
	local pitch = Body.get_rpy()[2]
	return T.trans(uTorso[1], uTorso[2], bH) * T.rotY(pitch)
end

function state.entry()
  print(state._NAME..' Entry' )
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
  headIKtarget = hcm.get_teleop_headik()
	doneHead = true
end

function state.update()
  -- print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  -- Grab the target
  local headIKtarget0 = hcm.get_teleop_headik()
	if headIKtarget~=headIKtarget0 then
		headIKtarget = headIKtarget0
		doneHead = false
	end
	if doneHead then return end

	local tfIKtorso = get_world_tf()*T.trans(unpack(headIKtarget))
	local vTarget = vector.new(T.position(tfIKtorso))
	local headAngles = vector.new{HT.ikineCam(unpack(vTarget))}
	local headNow = Body.get_head_command_position()
	local apprAng
  apprAng, doneHead = util.approachTol(headNow, headAngles, headSpeed, dt, headThresh)

  -- Update the motors
	Body.set_head_command_position(doneHead and headAngles or apprAng)

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
