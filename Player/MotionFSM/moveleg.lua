local moveleg={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

-- SJ: Shared library for 2D leg trajectory generation
-- So that we can reuse them for different controllers
-- Should we move it into util?

-- Foothold Generation parameters ---------------------------------------
local stanceLimitX = Config.walk.stanceLimitX or {-0.10 , 0.10}
local stanceLimitY = Config.walk.stanceLimitY or {0.09 , 0.20}
local stanceLimitA = Config.walk.stanceLimitA or {-0*math.pi/180, 40*math.pi/180}

-- Toe/heel overlap checking values
local footSizeX = Config.walk.footSizeX or {-0.05,0.05}
local stanceLimitMarginY = Config.walk.stanceLimitMarginY or 0.015
local stanceLimitY2 = 2* Config.walk.footY-stanceLimitMarginY
-------------------------------------------------------------------------

local footY    = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY

-- Gyro stabilization parameters
local ankleImuParamX = Config.walk.ankleImuParamX
local ankleImuParamY = Config.walk.ankleImuParamY
local kneeImuParamX  = Config.walk.kneeImuParamX
local hipImuParamY   = Config.walk.hipImuParamY

-- Hip sag compensation parameters
local hipRollCompensation = Config.walk.hipRollCompensation

-- Leg hardness parameters
local hardnessSupport = Config.walk.hardnessSupport or 0.7
local hardnessSwing   = Config.walk.hardnessSwing or 0.5

function moveleg.calculate_next_step(uLeft_now, uRight_now, supportLeg, uBody_diff)
  if supportLeg == 0 then    -- Left support
    -- Find the left support point
    local uSupport = util.pose_global({supportX, supportY, 0}, uLeft_now)
    -- Find the right foot destination
    local uRight_next = moveleg.step_destination_right(uBody_diff, uLeft_now, uRight_now)
    return uSupport, uLeft_now, uRight_next
  else    -- Right support
    -- Find the right support point
    local uSupport = util.pose_global({supportX, -supportY, 0}, uRight_now)
    -- Find the left foot destination
    local uLeft_next = moveleg.step_destination_left(uBody_diff, uLeft_now, uRight_now)
    return uSupport, uLeft_next, uRight_now
  end
end

function moveleg.calculate_next_torso(uLeft_next,uRight_next,supportLeg, shiftFactor)
  -- shiftFactor: How much should we shift final Torso pose?
  local u0 = util.se2_interpolate(.5, uLeft_next, uRight_next)
  -- NOTE: supportX and supportY are globals
  local uLeftSupport  = util.pose_global({supportX,  supportY, 0}, uLeft_next )
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight_next)
  if supportLeg==0 then --Left support
    local uTorso_next = util.se2_interpolate(shiftFactor, uLeftSupport, uRightSupport)
    return uTorso_next
  else
    local uTorso_next = util.se2_interpolate(shiftFactor, uRightSupport, uLeftSupport)
    return uTorso_next
  end
end


function moveleg.step_destination_left(vel, uLeft, uRight)
  local uLRFootOffset = vector.new({0,footY,0})

  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uLeftPredict = util.pose_global(uLRFootOffset, u2)
  local uLeftRight = util.pose_relative(uLeftPredict, uRight)
  -- Do not pidgeon toe, cross feet:
  --Check toe and heel overlap
  local toeOverlap  = -footSizeX[1] * uLeftRight[3]
  local heelOverlap = -footSizeX[2] * uLeftRight[3]
  local limitY = math.max(stanceLimitY[1],
  stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)
  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2])
  uLeftRight[2] = math.min(math.max(uLeftRight[2], limitY),stanceLimitY[2])
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2])

  return util.pose_global(uLeftRight, uRight)
end

function moveleg.step_destination_right(vel, uLeft, uRight)
  local uLRFootOffset = vector.new({0,footY,0})

  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uRightPredict = util.pose_global(-1*uLRFootOffset, u2)
  local uRightLeft = util.pose_relative(uRightPredict, uLeft)
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap  = footSizeX[1] * uRightLeft[3]
  local heelOverlap = footSizeX[2] * uRightLeft[3]
  local limitY = math.max(stanceLimitY[1], stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2])
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -limitY)
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1])

  return util.pose_global(uRightLeft, uLeft)
end

function moveleg.get_leg_compensation(supportLeg, phSingle, gyro_rpy,
    ankleShift, kneeShift, hipShift, initial_step)
  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])

  -- Ankle shift is filtered... thus a global
  ankleShift[1] = ankleShift[1] + ankleImuParamX[1]*(ankleShiftX-ankleShift[1])
  ankleShift[2] = ankleShift[2] + ankleImuParamY[1]*(ankleShiftY-ankleShift[2])

  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  kneeShift = kneeShift + kneeImuParamX[1]*(kneeShiftX-kneeShift)
  
  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])
  hipShift[2] = hipShift[2]+hipImuParamY[1]*(hipShiftY-hipShift[2])

  --TODO: Toe/heel lifting
  local toeTipCompensation = 0

  local delta_legs = vector.zeros(Body.nJointLLeg+Body.nJointRLeg)
  -- Change compensation in the beginning of the phase (first 10%)
  -- Saturate compensation afterwards
  -- Change compensation at the beginning of the phase (first 10%)
  -- Same sort of trapezoid at double->single->double support shape
  local phComp = 10 * math.min( phSingle, .1, 1-phSingle )

  --SJ: if initial step, hipRoll shouldn't be compensated
  if initial_step>0 then phComp = 0; end

  if supportLeg == 0 then
    -- Left support
    delta_legs[2] = hipShift[2] + hipRollCompensation*phComp
    delta_legs[4] = kneeShift
    delta_legs[5] = ankleShift[1]
    delta_legs[6] = ankleShift[2]
    -- right toe tip swing
    delta_legs[11] = toeTipCompensation*phComp--Lifting toetip
  else
    -- Right support
    delta_legs[8]  = hipShift[2] - hipRollCompensation*phComp
    delta_legs[10] = kneeShift
    delta_legs[11] = ankleShift[1]
    delta_legs[12] = ankleShift[2]
    -- left toe tip swing
    delta_legs[5] = toeTipCompensation*phComp--Lifting toetip
  end
  return delta_legs, ankleShift, kneeShift, hipShift
end

function moveleg.set_leg_positions(pLLeg, pRLeg, pTorso, supportLeg, delta_legs)
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg)
  qLegs = qLegs + delta_legs
  Body.set_lleg_command_position(qLegs)
  if supportLeg==0 then
    Body.set_lleg_hardness(hardnessSupport)
    Body.set_rleg_hardness(hardnessSwing)
  elseif supportLeg==1 then
    Body.set_lleg_hardness(hardnessSwing)
    Body.set_rleg_hardness(hardnessSupport)
  elseif supportLeg==2 then --Double-support phase (for standing still)
    Body.set_lleg_hardness(hardnessSupport)
    Body.set_rleg_hardness(hardnessSupport)
  end
end

return moveleg

