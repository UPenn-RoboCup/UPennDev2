local moveleg = {}
local Body   = require'Body'
local K = require'K_ffi'
local T = require'Transform'
local T6D = T.transform6D
local P6D = T.position6D
local util = require'util'
local approachTol = util.approachTol
local vector = require'vector'
local vnew = vector.new
require'mcm'

local torsoX = Config.walk.torsoX
local footY = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY

local slow_p_tolerance = {
  -- x, y, z
  1e-3, 1e-3, 1e-3,
  -- r, p, y
  1*DEG_TO_RAD, 1*DEG_TO_RAD, 1*DEG_TO_RAD
}

-- Zero bias to start
local legBiasR, legBiasL = vector.zeros(7), vector.zeros(7)
function moveleg.update_leg_bias()
	-- TODO: Fix the bias issue
  local legBias = 0*mcm.get_leg_bias()
  legBiasL = vector.slice(legBias, 1, 6)
  legBiasR = vector.slice(legBias, 7, 12)
end

-- How far away to tell the P controller to go in one step
local dqLegLimit = Config.stance.dqLegLimit
local dpLimitStance = Config.stance.dpLimitStance
--[[
pTorso:
pLLeg:
pRLeg:
dt:
--]]
local function set_lower_body_slowly(pTorso, pLLeg, pRLeg, dt)
  -- Where are we in the routine?
  local qL = Body.get_lleg_command_position()
  local qR = Body.get_rleg_command_position()
  local qLLegActual = qL - legBiasL
  local qRLegActual = qR - legBiasR
  -- How far away from the torso are the legs currently?
  local dpLLeg = P6D(T.inv(K.forward_lleg(qLLegActual)))
  local dpRLeg = P6D(T.inv(K.forward_rleg(qRLegActual)))	
  local pTorsoL = pLLeg + dpLLeg
  local pTorsoR = pRLeg + dpRLeg
  local pTorsoActual = (pTorsoL + pTorsoR) / 2
  -- Which torso to approach
  local pTorso_approach, doneTorso = approachTol(pTorsoActual, pTorso, dpLimitStance, dt, slow_p_tolerance)
  -- Where should the legs go?
  local qLLegTarget, qRLegTarget = K.inverse_legs(T6D(pLLeg), T6D(pRLeg), T6D(pTorso_approach))
	qLLegTarget = qLLegTarget + legBiasL
	qRLegTarget = qRLegTarget + legBiasR
  local qLLegMove, doneL = approachTol(qLLegActual, qLLegTarget, dqLegLimit, dt)
  local qRLegMove, doneR = approachTol(qRLegActual, qRLegTarget, dqLegLimit, dt)
  -- Set the legs
  Body.set_lleg_command_position(qLLegMove)
  Body.set_rleg_command_position(qRLegMove)
  -- Update our current situation
  mcm.set_stance_bodyTilt(pTorsoActual[5])
  mcm.set_stance_bodyHeightTarget(pTorsoActual[3])
  mcm.set_stance_bodyHeight(pTorsoActual[3])
  -- Return the status
  return doneL and doneR and doneTorso
end
moveleg.set_lower_body_slowly = set_lower_body_slowly

--[[
uTorso:
uLeft:
uRight:
zLeft:
zRight:
dq: Limit the amount of movement (unit: radians NOT radians per second)
]]
function moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight, dt)
  local uTorsoActual = util.pose_global({-torsoX, 0, 0}, uTorso)
  local pTorso = vnew{
    -- XYZ
    uTorsoActual[1],
    uTorsoActual[2],
    mcm.get_stance_bodyHeight(),
    -- RPY
    0,
    mcm.get_stance_bodyTilt(),
    uTorsoActual[3]
  }
  local pLLeg = vnew{uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]}
  local pRLeg = vnew{uRight[1],uRight[2],zRight,0,0,uRight[3]}
  return set_lower_body_slowly(pTorso, pLLeg, pRLeg, dt)
end

return moveleg
