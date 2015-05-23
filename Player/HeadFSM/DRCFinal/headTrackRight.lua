local state = {}
state._NAME = ...

local Body = require'Body'
local util = require'util'
local vector = require'vector'

local HT = require'libHeadTransform'
local T = require'Transform'
local movearm = require'movearm'
local forwardR = movearm.rPlanner.forward

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

local function get_world_tf()
	local bH = mcm.get_stance_bodyHeight()
	local uTorso = mcm.get_stance_uTorsoComp()
	local pitch = Body.get_rpy()[2]
	return T.trans(uTorso[1], uTorso[2], bH) * T.rotY(pitch)
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

	local qRArm = Body.get_rarm_position()
	local qWaist = Body.get_waist_position()
	local headNow = Body.get_head_command_position()

	local tfArm = forwardR(qRArm, qWaist)
	local tfWorld = get_world_tf()
	local tfHand = tfWorld * tfArm

	local headAngles = vector.new{HT.ikineCam(unpack(T.position(tfHand)))}
  local apprAng, doneHead = util.approachTol(headNow, headAngles, headSpeed, dt, headThresh)

	Body.set_head_command_position(apprAng)

  return doneHead and 'done'
end

function state.exit()
  print(state._NAME..' Exit')
end

return state
