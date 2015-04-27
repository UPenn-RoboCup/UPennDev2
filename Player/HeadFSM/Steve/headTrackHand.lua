local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'

local K = require'K_ffi'
local T = require'Transform'
local HT = require'libHeadTransform'

local t_entry, t_update

-- TODO: need to compensate the torso pose
local headSpeed = 5 * DEG_TO_RAD * vector.ones(2)
local headThresh = 1 * DEG_TO_RAD * vector.ones(2)

function state.entry()
  print(state._NAME..' Entry' ) 
  -- When entry was previously called
  local t_entry_prev = t_entry
  -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	local bH = mcm.get_stance_bodyHeight()
	local uTorso = mcm.get_stance_uTorsoComp()
	local pitch = Body.get_rpy()[2]

	local trGrip = T.trans(uTorso[1], uTorso[2], bH) * T.rotY(pitch)
		* K.forward_larm(Body.get_larm_position())
	local headAngles = vector.new{HT.ikineCam(unpack(T.position(trGrip)))}
	local headNow = Body.get_head_command_position()
  local apprAng, doneHead = util.approachTol(headNow, headAngles, headSpeed, dt, headThresh)
	
	Body.set_head_command_position(apprAng)

  return doneHead and 'done'
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
