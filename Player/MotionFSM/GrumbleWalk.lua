--THOR-OP specific (FOR 6DOF ARM)
local walk = {}
walk._NAME = 'GrumbleWalk'

local Config = require'Config'
local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
require'mcm'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber('Walk',true)

local t_entry, t_update

-- Walk Parameters
-- Stance and velocity limit values
local stanceLimitX = Config.walk.stanceLimitX or {-0.10 , 0.10}
local stanceLimitY = Config.walk.stanceLimitY or {0.09 , 0.20}
local stanceLimitA = Config.walk.stanceLimitA or {-0*math.pi/180, 40*math.pi/180}
local velLimitX = Config.walk.velLimitX or {-.06, .08}
local velLimitY = Config.walk.velLimitY or {-.06, .06}
local velLimitA = Config.walk.velLimitA or {-.4, .4}
local velDelta = Config.walk.velDelta or {.03,.015,.15}
local vaFactor = Config.walk.vaFactor or 0.6

local velXHigh = Config.walk.velXHigh or 0.06
local velDeltaXHigh = Config.walk.velDeltaXHigh or 0.01

--Toe/heel overlap checking values
local footSizeX = Config.walk.footSizeX or {-0.05,0.05}
local stanceLimitMarginY = Config.walk.stanceLimitMarginY or 0.015
local stanceLimitY2= 2* Config.walk.footY-stanceLimitMarginY

--OP default stance width: 0.0375*2 = 0.075
--Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
--Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

--Stance parameters
local bodyHeight = Config.walk.bodyHeight
local bodyTilt=Config.walk.bodyTilt or 0
local footX = Config.walk.footX
local footY = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY
local qLArm0=Config.walk.qLArm
local qRArm0=Config.walk.qRArm

--Hardness parameters
local hardnessSupport = Config.walk.hardnessSupport or 0.7
local hardnessSwing = Config.walk.hardnessSwing or 0.5

local hardnessArm0 = Config.walk.hardnessArm or 0.2
local hardnessArm = Config.walk.hardnessArm or 0.2

--Gait parameters
local tStep0 = Config.walk.tStep
local tStep = Config.walk.tStep
local tZmp = Config.walk.tZmp
local stepHeight0 = Config.walk.stepHeight
local stepHeight = Config.walk.stepHeight
local ph1Single = Config.walk.phSingle[1]
local ph2Single = Config.walk.phSingle[2]
local ph1Zmp,ph2Zmp=ph1Single,ph2Single

--Compensation parameters
local hipRollCompensation = Config.walk.hipRollCompensation
local ankleMod = Config.walk.ankleMod or {0,0}
local spreadComp = Config.walk.spreadComp or 0
local turnCompThreshold = Config.walk.turnCompThreshold or 0
local turnComp = Config.walk.turnComp or 0

--Gyro stabilization parameters
local ankleImuParamX = Config.walk.ankleImuParamX
local ankleImuParamY = Config.walk.ankleImuParamY
local kneeImuParamX = Config.walk.kneeImuParamX
local hipImuParamY = Config.walk.hipImuParamY
local armImuParamX = Config.walk.armImuParamX
local armImuParamY = Config.walk.armImuParamY

--Initial body swing 
local supportModYInitial = Config.walk.supportModYInitial or 0

----------------------------------------------------------
-- Walk state variables
----------------------------------------------------------

local uTorso = vector.new({supportX, 0, 0})
local uLeft = vector.new({0, footY, 0})
local uRight = vector.new({0, -footY, 0})

local pLLeg = vector.new({0, footY, 0, 0,0,0})
local pRLeg = vector.new({0, -footY, 0, 0,0,0})
local pTorso = vector.new({supportX, 0, bodyHeight, 0,bodyTilt,0})

local velCurrent = vector.new({0, 0, 0})
local velCommand = vector.new({0, 0, 0})
local velDiff = vector.new({0, 0, 0})

--ZMP exponential coefficients:
local aXP, aXN, aYP, aYN = 0, 0, 0, 0

--Gyro stabilization variables
local ankleShift = vector.new({0, 0})
local kneeShift = 0
local hipShift = vector.new({0,0})
local armShift = vector.new({0, 0})

local active = false
local started = false
local iStep0 = -1
local iStep = 0
local t0 = Body.get_time()
local tLastStep = Body.get_time()
local ph0=0
local ph=0
local phSingle = 0

local stopRequest = 2
local canWalkKick = 0 --Can we do walkkick with this walk code?
local initial_step=2
local upper_body_overridden = 0

--------------------
-- Local Functions
--------------------
local function step_torso(uLeft, uRight,shiftFactor)
  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeft)
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight)
  return util.se2_interpolate(shiftFactor, uLeftSupport, uRightSupport)
end

local function stance_reset() --standup/sitdown/falldown handling
  print"Stance has been reset!"
  uLeft = util.pose_global(vector.new({-supportX, footY, 0}),uTorso)
  uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso)
  uLeft1, uLeft2 = uLeft, uLeft
  uRight1, uRight2 = uRight, uRight
  uTorso1, uTorso2 = uTorso, uTorso
  uSupport = uTorso
  tLastStep = Body.get_time()
  walkKickRequest = 0
  iStep0 = -1
  iStep = 0
  current_step_type=0
  motion_playing = 0
  uLRFootOffset = vector.new({0,footY,0})
end

local function update_still()
  uTorso = step_torso(uLeft, uRight,0.5)

  pTorso[4], pTorso[5],pTorso[6] = 0,bodyTilt,0

  uTorsoActual = util.pose_global(vector.new({-footX,0,0}),uTorso)

  pTorso[6] = pTorso[6]+ uTorsoActual[3]
  pTorso[1], pTorso[2] = uTorsoActual[1],uTorsoActual[2] 

  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3]
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3]
  
  qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg)
  motion_legs(qLegs)
  motion_arms()
end

local function motion_legs(qLegs)
  phComp = math.min(1, phSingle/.1, (1-phSingle)/.1)

  --Ankle stabilization using gyro feedback
  imuGyr = Body.get_sensor_gyro()

  gyro_roll0=imuGyr[1]
  gyro_pitch0=imuGyr[2]

  --get effective gyro angle considering body angle offset
  if not active then --double support
    yawAngle = (uLeft[3]+uRight[3])/2-uTorsoActual[3]
  elseif supportLeg == 0 then  -- Left support
    yawAngle = uLeft[3]-uTorsoActual[3]
  elseif supportLeg==1 then
    yawAngle = uRight[3]-uTorsoActual[3]
  end
  gyro_roll = gyro_roll0*math.cos(yawAngle) -gyro_pitch0* math.sin(yawAngle)
  gyro_pitch = gyro_pitch0*math.cos(yawAngle) -gyro_roll0* math.sin(yawAngle)

  armShiftX=util.procFunc(gyro_pitch*armImuParamY[2],armImuParamY[3],armImuParamY[4])
  armShiftY=util.procFunc(gyro_roll*armImuParamY[2],armImuParamY[3],armImuParamY[4])

  ankleShiftX=util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  ankleShiftY=util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])
  kneeShiftX=util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])

  ankleShift[1]=ankleShift[1]+ankleImuParamX[1]*(ankleShiftX-ankleShift[1])
  ankleShift[2]=ankleShift[2]+ankleImuParamY[1]*(ankleShiftY-ankleShift[2])
  kneeShift=kneeShift+kneeImuParamX[1]*(kneeShiftX-kneeShift)
  hipShift[2]=hipShift[2]+hipImuParamY[1]*(hipShiftY-hipShift[2])
  armShift[1]=armShift[1]+armImuParamX[1]*(armShiftX-armShift[1])
  armShift[2]=armShift[2]+armImuParamY[1]*(armShiftY-armShift[2])

  --TODO: Toe/heel lifting
  toeTipCompensation = 0


  if not active then --Double support, standing still
    --qLegs[2] = qLegs[2] + hipShift[2]    --Hip roll stabilization
    qLegs[4] = qLegs[4] + kneeShift    --Knee pitch stabilization
    qLegs[5] = qLegs[5]  + ankleShift[1]    --Ankle pitch stabilization
    --qLegs[6] = qLegs[6] + ankleShift[2]    --Ankle roll stabilization

    --qLegs[8] = qLegs[8]  + hipShift[2]    --Hip roll stabilization
    qLegs[10] = qLegs[10] + kneeShift    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1]    --Ankle pitch stabilization
    --qLegs[12] = qLegs[12] + ankleShift[2]    --Ankle roll stabilization

  elseif supportLeg == 0 then  -- Left support
    qLegs[2] = qLegs[2] + hipShift[2]    --Hip roll stabilization
    qLegs[4] = qLegs[4] + kneeShift    --Knee pitch stabilization
    qLegs[5] = qLegs[5]  + ankleShift[1]    --Ankle pitch stabilization
    qLegs[6] = qLegs[6] + ankleShift[2]    --Ankle roll stabilization

    qLegs[11] = qLegs[11]  + toeTipCompensation*phComp--Lifting toetip

--    if initial_step==0 then 
    if true then
      qLegs[2] = qLegs[2] + hipRollCompensation*phComp --Hip roll compensation
    end
  else
    qLegs[8] = qLegs[8]  + hipShift[2]    --Hip roll stabilization
    qLegs[10] = qLegs[10] + kneeShift    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1]    --Ankle pitch stabilization
    qLegs[12] = qLegs[12] + ankleShift[2]    --Ankle roll stabilization

    qLegs[5] = qLegs[5]  + toeTipCompensation*phComp--Lifting toetip
--    if initial_step==0 then 
    if true then
      qLegs[8] = qLegs[8] - hipRollCompensation*phComp--Hip roll compensation
    end
  end

  Body.set_lleg_command_position(qLegs)
end

local function motion_arms()
  if upper_body_overridden>0 then return end
  
  local qLArmActual = vector.zeros(Body.nJointLArm)
  local qRArmActual = vector.zeros(Body.nJointLArm)

  qLArmActual[1],qLArmActual[2]=qLArm0[1]+armShift[1],qLArm0[2]+armShift[2]
  qRArmActual[1],qRArmActual[2]=qRArm0[1]+armShift[1],qRArm0[2]+armShift[2]

  if upper_body_overridden>0 or motion_playing>0 then
    qLArmActual[1],qLArmActual[2],qLArmActual[3]=qLArmOR[1],qLArmOR[2],qLArmOR[3]
    qRArmActual[1],qRArmActual[2],qRArmActual[3]=qRArmOR[1],qRArmOR[2],qRArmOR[3]
    qLArmActual[4],qLArmActual[5],qLArmActual[6]=qLArmOR[4],qLArmOR[5],qLArmOR[6]
    qRArmActual[4],qRArmActual[5],qRArmActual[6]=qRArmOR[4],qRArmOR[5],qRArmOR[6]
  end
--  qLArmActual[3]=qLArm0[3]
--  qRArmActual[3]=qRArm0[3]
  Body.set_larm_command_position(qLArmActual)
  Body.set_rarm_command_position(qRArmActual)
end

local function step_left_destination(vel, uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uLeftPredict = util.pose_global(uLRFootOffset, u2)
  local uLeftRight = util.pose_relative(uLeftPredict, uRight)
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap= -footSizeX[1]*uLeftRight[3]
  local heelOverlap= -footSizeX[2]*uLeftRight[3]
  local limitY = math.max(stanceLimitY[1],
  stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2])
  uLeftRight[2] = math.min(math.max(uLeftRight[2], limitY),stanceLimitY[2])
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2])

  return util.pose_global(uLeftRight, uRight)
end

local function step_right_destination(vel, uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uRightPredict = util.pose_global(-1*uLRFootOffset, u2)
  local uRightLeft = util.pose_relative(uRightPredict, uLeft)
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap= footSizeX[1]*uRightLeft[3]
  local heelOverlap= footSizeX[2]*uRightLeft[3]
  local limitY = math.max(stanceLimitY[1],
  stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2])
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -limitY)
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1])

  return util.pose_global(uRightLeft, uLeft)
end

local function update_velocity()
  velDiff[1]= math.min(math.max(velCommand[1]-velCurrent[1],
    -velDelta[1]),velDelta[1])
  velDiff[2]= math.min(math.max(velCommand[2]-velCurrent[2],
  -velDelta[2]),velDelta[2])
  velDiff[3]= math.min(math.max(velCommand[3]-velCurrent[3],
  -velDelta[3]),velDelta[3])

  velCurrent[1] = velCurrent[1]+velDiff[1]
  velCurrent[2] = velCurrent[2]+velDiff[2]
  velCurrent[3] = velCurrent[3]+velDiff[3]

  if initial_step>0 then
    velCurrent=vector.new({0,0,0})
    initial_step=initial_step-1
  end

  -- Save the updated velocity to sahred memory
  mcm.set_walk_current_vel(velCurrent)

  --  print(string.format("VEL:%.2f,%.2f,%.2f",unpack(velCurrent)))
end

local function zmp_solve(zs, z1, z2, x1, x2)
  --[[
  Solves ZMP equation:
  x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
  where the ZMP point is piecewise linear:
  z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
  --]]
  local T1 = tStep*ph1Zmp
  local T2 = tStep*ph2Zmp
  local m1 = (zs-z1)/T1
  local m2 = -(zs-z2)/(tStep-T2)
  local c1 = x1-z1+tZmp*m1*math.sinh(-T1/tZmp)
  local c2 = x2-z2+tZmp*m2*math.sinh((tStep-T2)/tZmp)
  local expTStep = math.exp(tStep/tZmp)
  local aP = (c2 - c1/expTStep)/(expTStep-1/expTStep)
  local aN = (c1*expTStep - c2)/(expTStep-1/expTStep)
  return aP, aN
end

--Finds the necessary COM for stability and returns it
local function zmp_com(ph)
  local com = vector.new({0, 0, 0})
  local expT = math.exp(tStep*ph/tZmp)
  com[1] = uSupport[1] + aXP*expT + aXN/expT
  com[2] = uSupport[2] + aYP*expT + aYN/expT
  if ph < ph1Zmp then
    com[1] = com[1] + m1X*tStep*(ph-ph1Zmp)
    -tZmp*m1X*math.sinh(tStep*(ph-ph1Zmp)/tZmp)
    com[2] = com[2] + m1Y*tStep*(ph-ph1Zmp)
    -tZmp*m1Y*math.sinh(tStep*(ph-ph1Zmp)/tZmp)
  elseif (ph > ph2Zmp) then
    com[1] = com[1] + m2X*tStep*(ph-ph2Zmp)
    -tZmp*m2X*math.sinh(tStep*(ph-ph2Zmp)/tZmp)
    com[2] = com[2] + m2Y*tStep*(ph-ph2Zmp)
    -tZmp*m2Y*math.sinh(tStep*(ph-ph2Zmp)/tZmp)
  end
  --com[3] = .5*(uLeft[3] + uRight[3])
  --Linear speed turning
  com[3] = ph* (uLeft2[3]+uRight2[3])/2 + (1-ph)* (uLeft1[3]+uRight1[3])/2
  return com
end

local function foot_phase(ph)
  -- Computes relative x,z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  return xf, zf
end

---------------------------
-- Handle walk requests --
---------------------------
local walk_requests = {}
-- Setting velocity
walk_requests.set_velocity = function()

  -- Grab from the shared memory
  vx,vy,va = unpack(mcm.get_walk_vel())

  --Filter the commanded speed
  vx = math.min(math.max(vx,velLimitX[1]),velLimitX[2])
  vy = math.min(math.max(vy,velLimitY[1]),velLimitY[2])
  va = math.min(math.max(va,velLimitA[1]),velLimitA[2])

  --Slow down when turning
  vFactor = 1-math.abs(va)/vaFactor

  local stepMag=math.sqrt(vx^2+vy^2)
  local magFactor=math.min(velLimitX[2]*vFactor,stepMag)/(stepMag+0.000001)

  velCommand[1]=vx*magFactor
  velCommand[2]=vy*magFactor
  velCommand[3]=va

  velCommand[1] = math.min(math.max(velCommand[1],velLimitX[1]),velLimitX[2])
  velCommand[2] = math.min(math.max(velCommand[2],velLimitY[1]),velLimitY[2])
  velCommand[3] = math.min(math.max(velCommand[3],velLimitA[1]),velLimitA[2])
end
walk_requests.start = function()
  print( util.color('Walk Start','green'))
  stopRequest = 0
  if not active then
    active = true
    started = false
    iStep0 = -1
    t0 = Body.get_time()
    tLastStep = Body.get_time()
    initial_step = 2
  end
end
walk_requests.stop = function()
  print( util.color('Walk Stop','red'))
  --Always stops with feet together (which helps kicking)
  stopRequest = math.max(1,stopRequest)
  -- stopRequest = 2 --Stop w/o feet together
end

---------------------------
-- State machine methods --
---------------------------

function walk.entry()
  print(walk._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  --SJ: now we always assume that we start walking with feet together
  --Because joint readings are not always available with darwins
  stance_reset()
  --Place arms in appropriate position at sides

  if upper_body_overridden==0 then
    Body.set_larm_command_position(qLArm0)
    Body.set_rarm_command_position(qRArm0)
    -- TODO: hardness
    Body.set_larm_hardness(hardnessArm)
    Body.set_rarm_hardness(hardnessArm)
  end
  Body.set_waist_command_position(0)
  Body.set_waist_hardness(1.0)
  mcm.set_walk_bipedal(1)
  
  -- Start walking
  walk_requests.start()
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

  -- Parse mcm events
  -- TODO: Maybe use a WalkFSM channel?
  -- Check for out of process events in non-blocking
  local event, has_more
  repeat
    event, has_more = evts:receive(true)
    if type(event)=='string' then
      local request = walk_requests[event]
      --print( util.color('Walk Event:','green'),event)
      if request then request() end
    end
  until not has_more

  -- TODO: Don't run update 
  -- if the robot is sitting or standing
  if not active then 
    walk.update_still()
    return 
  end

  if not started then
    started = true
    tLastStep = Body.get_time()
  end
  ph0 = ph

  --SJ: Variable tStep support for walkkick
  ph = (t-tLastStep)/tStep
  if ph>1 then
    iStep=iStep+1
    ph=ph-math.floor(ph)
    tLastStep=tLastStep+tStep
  end

  --Stop when stopping sequence is done
  if iStep>iStep0 and stopRequest==2 then
    stopRequest = 0
    active = false
    return "stop"
  end

  -- New step
  if iStep>iStep0 then
    update_velocity()
    iStep0 = iStep
    supportLeg = iStep % 2 -- 0 for left support, 1 for right support
    uLeft1 = uLeft2
    uRight1 = uRight2
    uTorso1 = uTorso2

    supportMod = {0,0} --Support Point modulation for walkkick
    shiftFactor = 0.5 --How much should we shift final Torso pose?

    if walkKickRequest==0 then
      if stopRequest==1 then --Final step
        stopRequest=2
        velCurrent=vector.new({0,0,0})
        velCommand=vector.new({0,0,0})
        if supportLeg == 0 then        -- Left support
          uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1)
        else        -- Right support
          uLeft2 = util.pose_global(2*uLRFootOffset, uRight1)
        end
      else --Normal walk, advance steps
        tStep=tStep0 
        if supportLeg == 0 then-- Left support
          uRight2 = step_right_destination(velCurrent, uLeft1, uRight1)
        else  -- Right support
          uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1)
        end
      end
    end

    uTorso2 = step_torso(uLeft2, uRight2,shiftFactor)

    --Adjustable initial step body swing
    if initial_step>0 then 
      if supportLeg == 0 then --LS
        supportMod[2]=supportModYInitial
      else --RS
        supportMod[2]=-supportModYInitial
      end
    end

    --Apply velocity-based support point modulation for uSupport
    if supportLeg == 0 then --LS
      local uLeftTorso = util.pose_relative(uLeft1,uTorso1)
      local uTorsoModded = util.pose_global(
        vector.new({supportMod[1],supportMod[2],0}),uTorso)
      local uLeftModded = util.pose_global (uLeftTorso,uTorsoModded) 
      uSupport = util.pose_global({supportX, supportY, 0},uLeftModded)
      Body.set_lleg_hardness(hardnessSupport)
      Body.set_rleg_hardness(hardnessSwing)
    else --RS
      local uRightTorso = util.pose_relative(uRight1,uTorso1)
      local uTorsoModded = 
        util.pose_global(vector.new({supportMod[1],supportMod[2],0}),uTorso)
      local uRightModded = util.pose_global (uRightTorso,uTorsoModded) 
      uSupport = util.pose_global({supportX, -supportY, 0}, uRightModded)
      Body.set_lleg_hardness(hardnessSwing)
      Body.set_rleg_hardness(hardnessSupport)
    end

    --Compute ZMP coefficients
    m1X = (uSupport[1]-uTorso1[1])/(tStep*ph1Zmp)
    m2X = (uTorso2[1]-uSupport[1])/(tStep*(1-ph2Zmp))
    m1Y = (uSupport[2]-uTorso1[2])/(tStep*ph1Zmp)
    m2Y = (uTorso2[2]-uSupport[2])/(tStep*(1-ph2Zmp))
    aXP, aXN = zmp_solve(uSupport[1], uTorso1[1], uTorso2[1],
    uTorso1[1], uTorso2[1])
    aYP, aYN = zmp_solve(uSupport[2], uTorso1[2], uTorso2[2],
    uTorso1[2], uTorso2[2])

  end --End new step

  xFoot, zFoot = foot_phase(ph)  
  if initial_step>0 then zFoot=0  end --Don't lift foot at initial step
  pLLeg[3], pRLeg[3] = 0
  if supportLeg == 0 then    -- Left support
    uRight = util.se2_interpolate(xFoot, uRight1, uRight2)
    pRLeg[3] = stepHeight*zFoot
  else    -- Right support
    uLeft = util.se2_interpolate(xFoot, uLeft1, uLeft2)
    pLLeg[3] = stepHeight*zFoot
  end
  uTorsoOld=uTorso
  uTorso = zmp_com(ph)

  pTorso[4], pTorso[5],pTorso[6] = 0,bodyTilt,0

  uTorsoActual = util.pose_global(vector.new({-footX,0,0}),uTorso)

  pTorso[1], pTorso[2] = uTorsoActual[1],uTorsoActual[2] 
  pTorso[6] = pTorso[6]+ uTorsoActual[3]
  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3]
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3]

  qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg)
  motion_legs(qLegs)
  motion_arms()
end -- walk.update

function walk.exit()
  print(state._NAME..' Exit')
end

function walk.get_odometry(u0)
  u0 = u0 or vector.new({0, 0, 0})
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  return util.pose_relative(uFoot, u0), uFoot
end

function walk.get_body_offset()
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  return util.pose_relative(uTorso, uFoot)
end

function walk.doWalkKickLeft()
end

function walk.doWalkKickRight()
end

function walk.doWalkKickLeft2()
end

function walk.doWalkKickRight2()
end

function walk.doSideKickLeft()
end

function walk.doSideKickRight()
end

function walk.zero_velocity()
end

function walk.doPunch(punchtype)
end

function walk.switch_stance(stance)
end
function walk.upper_body_override(qL, qR, bR)
end
function walk.upper_body_override_on()
  upper_body_overridden=1
end
function walk.upper_body_override_off()
  upper_body_overridden=0
end

return walk