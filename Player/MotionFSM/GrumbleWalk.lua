--THOR-OP specific (FOR 6DOF ARM)
local walk = {}
walk._NAME = 'GrumbleWalk'

local Config = require'Config'
local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local libZMP = require'libZMP'
local zmp_solver
require'mcm'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber('Walk',true)

-- Keep track of important times
local t_entry, t_update, t_last_step

----------------------------------------------------------
-- Walk Parameters
--
-- These are set one and read everywhere.
-- Used for conveinence
----------------------------------------------------------
-- Stance limits used in step_destination_* functions
local stanceLimitX = Config.walk.stanceLimitX or {-0.10 , 0.10}
local stanceLimitY = Config.walk.stanceLimitY or {0.09 , 0.20}
local stanceLimitA = Config.walk.stanceLimitA or {-0*math.pi/180, 40*math.pi/180}
-- Toe/heel overlap checking values
local footSizeX = Config.walk.footSizeX or {-0.05,0.05}
local stanceLimitMarginY = Config.walk.stanceLimitMarginY or 0.015
local stanceLimitY2 = 2* Config.walk.footY-stanceLimitMarginY

-- Velocity limits used in update_velocity function
local velLimitX = Config.walk.velLimitX or {-.06, .08}
local velLimitY = Config.walk.velLimitY or {-.06, .06}
local velLimitA = Config.walk.velLimitA or {-.4, .4}
local velDelta  = Config.walk.velDelta or {.03,.015,.15}
local vaFactor  = Config.walk.vaFactor or 0.6

-- Stance parameters
local bodyTilt = Config.walk.bodyTilt or 0
local torsoX   = Config.walk.torsoX
local footY    = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY
local qLArm0   = Config.walk.qLArm
local qRArm0   = Config.walk.qRArm

-- Hardness parameters
local hardnessSupport = Config.walk.hardnessSupport or 0.7
local hardnessSwing   = Config.walk.hardnessSwing or 0.5
local hardnessArm     = Config.walk.hardnessArm or 0.2
local hardnessArm0    = hardnessArm

--Gait parameters
local tZmp   = Config.walk.tZmp
local tStep0 = Config.walk.tStep
local tStep  = tStep0
local stepHeight  = Config.walk.stepHeight
local ph1Single,ph2Single = unpack(Config.walk.phSingle)
local ph1Zmp,ph2Zmp = ph1Single, ph2Single

-- Gyro stabilization parameters
local ankleImuParamX = Config.walk.ankleImuParamX
local ankleImuParamY = Config.walk.ankleImuParamY
local kneeImuParamX  = Config.walk.kneeImuParamX
local hipImuParamY   = Config.walk.hipImuParamY
local armImuParamX   = Config.walk.armImuParamX
local armImuParamY   = Config.walk.armImuParamY

-- Compensation parameters
local hipRollCompensation = Config.walk.hipRollCompensation
-- Initial body swing 
local supportModYInitial = Config.walk.supportModYInitial or 0

----------------------------------------------------------
-- Walk state variables
--
-- These are continuously updated on each update
----------------------------------------------------------
local uTorso = vector.new({supportX, 0, 0})
local uLeft  = vector.new({0,  footY, 0})
local uRight = vector.new({0, -footY, 0})
-- Save the velocity between update cycles
local velCurrent = vector.new({0, 0, 0})

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local ankleShift = vector.new({0, 0})
local kneeShift  = 0
local hipShift   = vector.new({0, 0})
local armShift   = vector.new({0, 0})

-- Still have an initial step for now
local initial_step = 2
local upper_body_overridden = 0

--------------------
-- Local Functions
--------------------
-- Take footsteps, and convert to torso positions
local function step_torso( uLeft, uRight, shiftFactor )
  -- shiftFactor: How much should we shift final Torso pose?
  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeft)
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight)
  return util.se2_interpolate(shiftFactor, uLeftSupport, uRightSupport)
end

local function get_gyro_feedback()
  local body_yaw
  if supportLeg == 0 then  -- Left support
    body_yaw = uLeft[3]-uTorsoActual[3]
  else
    body_yaw = uRight[3]-uTorsoActual[3]
  end
  -- Ankle stabilization using gyro feedback
  local gyro_roll0, gyro_pitch0, gyro_yaw0= unpack(Body.get_sensor_gyro())
  -- Get effective gyro angle considering body yaw offset
  -- Rotate the Roll and pitch about the intended body yaw
  local gyro_roll  = gyro_roll0  * math.cos(body_yaw) - gyro_pitch0 * math.sin(body_yaw)
  local gyro_pitch = gyro_pitch0 * math.cos(body_yaw) - gyro_roll0  * math.sin(body_yaw)
  -- Give these parameters
  return gyro_roll, gyro_pitch, gyro_yaw
end

local function get_leg_feedback(phSingle,gyro_roll,gyro_pitch,gyro_yaw)

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])
  ankleShift[1] = ankleShift[1]+ankleImuParamX[1]*(ankleShiftX-ankleShift[1])
  ankleShift[2] = ankleShift[2]+ankleImuParamY[1]*(ankleShiftY-ankleShift[2])

  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  kneeShift = kneeShift+kneeImuParamX[1]*(kneeShiftX-kneeShift)
  
  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])
  hipShift[2]=hipShift[2]+hipImuParamY[1]*(hipShiftY-hipShift[2])

  --TODO: Toe/heel lifting
  local toeTipCompensation = 0

  local delta_legs = vector.zeros(Body.nJointLLeg+Body.nJointRLeg)

  if supportLeg == 0 then
    -- Left support
    local phComp = math.min( 1, phSingle/.1, (1-phSingle)/.1 )
    delta_legs[2] = hipShift[2] + hipRollCompensation*phComp
    delta_legs[4] = kneeShift
    delta_legs[5] = ankleShift[1]
    delta_legs[6] = ankleShift[2]
    -- right toe tip swing
    delta_legs[11] = toeTipCompensation*phComp--Lifting toetip
  else
    -- Right support
    local phComp = math.min( 1, phSingle/.1, (1-phSingle)/.1 )
    delta_legs[8]  = hipShift[2] - hipRollCompensation*phComp
    delta_legs[10] = kneeShift
    delta_legs[11] = ankleShift[1]
    delta_legs[12] = ankleShift[2]
    -- left toe tip swing
    delta_legs[5] = toeTipCompensation*phComp--Lifting toetip
  end

  return delta_legs

end

local function get_arm_feedback(gyro_roll,gyro_pitch,gyro_yaw)
  -- Arm feedback amount in X/Y directions
  local armShiftX=util.procFunc(gyro_pitch*armImuParamX[2],armImuParamX[3],armImuParamX[4])
  local armShiftY=util.procFunc(gyro_roll*armImuParamY[2],armImuParamY[3],armImuParamY[4])
  armShift[1]=armShift[1]+armImuParamX[1]*(armShiftX-armShift[1])
  armShift[2]=armShift[2]+armImuParamY[1]*(armShiftY-armShift[2])
  -- Arm delta to apply to the joints
  local delta_arms = vector.zeros(Body.nJointLArm)
  -- First arm joint is roll
  delta_arms[1] = armShift[1]
  -- Second arm joint
  delta_arms[2] = armShift[2]
  -- TODO: Use IK to shift arms
  return delta_arms
end

local function step_destination_left(vel, uLeft, uRight)
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

local function step_destination_right(vel, uLeft, uRight)
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

local function update_velocity()
  -- If our first step(s), then use zero velocity
  if initial_step>0 then
    velCurrent=vector.new{0,0,0}
    initial_step=initial_step-1
    return
  end
  -- Grab from the shared memory the desired walking speed
  local vx,vy,va = unpack(mcm.get_walk_vel())
  --Filter the commanded speed
  vx = math.min(math.max(vx,velLimitX[1]),velLimitX[2])
  vy = math.min(math.max(vy,velLimitY[1]),velLimitY[2])
  va = math.min(math.max(va,velLimitA[1]),velLimitA[2])
  -- Find the magnitude of the velocity
  local stepMag = math.sqrt(vx^2+vy^2)
  --Slow down when turning
  local vFactor   = 1-math.abs(va)/vaFactor
  local magFactor = math.min(velLimitX[2]*vFactor,stepMag)/(stepMag+0.000001)
  -- Limit the forwards and backwards velocity
  vx = math.min(math.max(vx*magFactor,velLimitX[1]),velLimitX[2])
  vy = math.min(math.max(vy*magFactor,velLimitY[1]),velLimitY[2])
  va = math.min(math.max(va,velLimitA[1]),velLimitA[2])
  -- Check the change in the velocity
  local velDiff = vector.new({vx,vy,va}) - velCurrent
  -- Limit the maximum velocity change PER STEP
  velDiff[1] = util.procFunc(velDiff[1],0,velDelta[1])
  velDiff[2] = util.procFunc(velDiff[2],0,velDelta[2])
  velDiff[3] = util.procFunc(velDiff[3],0,velDelta[3])
  -- Update the current velocity command
  velCurrent = velCurrent + velDiff
  -- Save the updated velocity to shared memory
  mcm.set_status_velocity(velCurrent)
  -- Print a debugging message
  local debug = string.format("%g, %g, %g -> %g, %g, %g",vx,vy,va,unpack(velCurrent))
  print( util.color('Walk velocity','blue'), debug )
end

local function calculate_step()
  
  -- Advance the steps to find the next desired torso position
  -- Increment the step index
  iStep = iStep + 1
  -- supportLeg: 0 for left support, 1 for right support
  supportLeg = iStep % 2

  -- Reset the time between steps
  tStep = tStep0

  -- Save the previous desired foot positions as the current foot positions
  -- TODO: Some feedback could get the uLeft/uRight perfectly correct
  uLeft1  = uLeft2
  uRight1 = uRight2
  if supportLeg == 0 then
    -- Left support
    uRight2 = step_destination_right(velCurrent, uLeft1, uRight1)
  else
    -- Right support
    uLeft2  = step_destination_left(velCurrent, uLeft1, uRight1)
  end
  
  -- This torso position is the prior desired torso position
  uTorso1 = uTorso2
  -- This is the next desired torso position
  uTorso2 = step_torso(uLeft2, uRight2, 0.5)

  --Support Point modulation for walkkick
  local supportMod = {0,0}
  -- Adjustable initial step body swing
  if initial_step>0 then 
    if supportLeg == 0 then
      -- left
      supportMod[2]=supportModYInitial
    else
      -- right
      supportMod[2]=-supportModYInitial
    end
  end

  -- Apply velocity-based support point modulation for uSupport
  if supportLeg == 0 then
    -- left
    local uLeftTorso = util.pose_relative(uLeft1,uTorso1)
    local uTorsoModded = util.pose_global(
      vector.new({supportMod[1],supportMod[2],0}),uTorso)
    local uLeftModded = util.pose_global (uLeftTorso,uTorsoModded) 
    uSupport = util.pose_global({supportX, supportY, 0},uLeftModded)
    Body.set_lleg_hardness(hardnessSupport)
    Body.set_rleg_hardness(hardnessSwing)
  else
    -- right
    local uRightTorso = util.pose_relative(uRight1,uTorso1)
    local uTorsoModded = 
      util.pose_global(vector.new({supportMod[1],supportMod[2],0}),uTorso)
    local uRightModded = util.pose_global (uRightTorso,uTorsoModded) 
    uSupport = util.pose_global({supportX, -supportY, 0}, uRightModded)
    Body.set_lleg_hardness(hardnessSwing)
    Body.set_rleg_hardness(hardnessSupport)
  end

  zmp_solver:compute(uSupport,uTorso1,uTorso2)

end

---------------------------
-- Handle walk requests --
---------------------------
local walk_requests = {}
-- Setting velocity
walk_requests.some_request = function()
  print('some request')
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
  
  -- Reset our velocity
  velCurrent = vector.new{0,0,0}
  mcm.set_walk_vel{0,0,0}

  -- Make our ZMP solver
  zmp_solver = libZMP.new_solver({
    ['tStep'] = tStep,
    ['tZMP'] = tZmp,
    ['start_phase'] = ph1Single,
    ['finish_phase'] = ph2Single,
  })

  -- SJ: now we always assume that we start walking with feet together
  -- Because joint readings are not always available with darwins
  -- TODO: Use shared memory readings, or calculate upon entry
  -- (Re)Set our local variables to the current uFoot positions
  uLeft  = util.pose_global(vector.new({-supportX, footY, 0}),uTorso)
  uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso)
  uLeft1, uLeft2 = uLeft, uLeft
  uRight1, uRight2 = uRight, uRight
  uTorso1, uTorso2 = uTorso, uTorso
  uSupport = uTorso
  
  -- Compute initial zmp from these foot positions
  zmp_solver:compute( uSupport,uTorso1, uTorso2 )

    -- Zero the step index
  iStep = 1
  uLRFootOffset = vector.new({0,footY,0})

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

  -- Entry is now the start point
  t_last_step = Body.get_time()
  initial_step = 2

end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

  ------------------------------------------
  -- Check for out of process events
  -- TODO: May not need this...
  local event, has_more
  repeat
    event, has_more = evts:receive(true)
    if type(event)=='string' then
      local request = walk_requests[event]
      --print( util.color('Walk Event:','green'),event)
      if request then request() end
    end
  until not has_more
  ------------------------------------------

  -- SJ: Variable tStep support for walkkick
  -- Grab our phase for the current tStep
  local ph = (t-t_last_step)/tStep
  if ph>1 then
    -- Wrap around the phase
    ph = ph % 1
    -- Update the velocity via a filter
    update_velocity()
    -- Calculate the next step
    calculate_step()
    -- Update t_last_step
    t_last_step = Body.get_time()
  end

  -- Where does zmp think the torso should be?
  uTorso = zmp_solver:get_com(ph)
  -- Where does zmp think the swing foot should be?
  local xFoot, zFoot, phSingle = zmp_solver:get_foot(ph)  
  --Don't lift foot at initial step
  if initial_step>0 then zFoot = 0 end

  -- Begin to solve for our leg positions
  local pLLeg = vector.new{0, footY,0, 0,0,0}
  local pRLeg = vector.new{0,-footY,0, 0,0,0}
  if supportLeg == 0 then
    -- Left support
    uRight = util.se2_interpolate(xFoot, uRight1, uRight2)
    pRLeg[3] = stepHeight*zFoot
  else
    -- Right support
    uLeft = util.se2_interpolate(xFoot, uLeft1, uLeft2)
    pLLeg[3] = stepHeight*zFoot
  end

  -- Adjust the yaw angle of the center of mass
  --com[3] = .5*(uLeft[3] + uRight[3])
  -- Linear speed turning
  uTorso[3] = ph*(uLeft2[3]+uRight2[3])/2 + (1-ph)*(uLeft1[3]+uRight1[3])/2
  
  -- The reference frame is from the right foot, so reframe the torso
  uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  -- Grab gyro feedback for these joint angles
  local r,p,y = get_gyro_feedback()

   -- Calculate the next desired torso position
  local pTorso = vector.new({supportX, 0, Config.walk.bodyHeight, 0,bodyTilt,0})
  -- The position of the torso, in Transform format is here
  pTorso[1], pTorso[2] = uTorsoActual[1], uTorsoActual[2] 
  -- Add the default angle of the torso in RPY format
  pTorso[4], pTorso[5],pTorso[6] = 0, bodyTilt, 0
  -- The yaw angle should include the default body angle and the yaw swing of the torso
  pTorso[6] = pTorso[6] + uTorsoActual[3]

  -- Find the transform of the legs, from the torso information
  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3]
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3]
  -- Solve the IK for these transforms
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg)
  -- Make the motion commands for the legs
  local leg_feedback = get_leg_feedback(phSingle,r,p,y)
  qLegs = qLegs + leg_feedback
  -- Send the leg commands
  Body.set_lleg_command_position(qLegs)
  -- Add arm motion
  if upper_body_overridden==0 then
    local arm_feedback = get_arm_feedback(r,p,y)
    Body.set_larm_command_position(qLArm0+arm_feedback)
    Body.set_rarm_command_position(qRArm0+arm_feedback)
  end
  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------

end -- walk.update

function walk.exit()
  print(walk._NAME..' Exit')

  -- These are the desired stopping foot placements
  if supportLeg == 0 then
    -- Left support
    uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1)
  else
    -- Right support
    uLeft2 = util.pose_global(2*uLRFootOffset, uRight1)
  end

end

return walk