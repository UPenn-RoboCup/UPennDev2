--New, cleaned-up humblewalk 
--Got rid of any non-used codes (including walkkick)
--Arm movement is turned off (only handled by arm FSM)

local walk = {}
walk._NAME = ...

local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local moveleg = require'moveleg'
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

--Gait parameters
local stepHeight  = Config.walk.stepHeight

----------------------------------------------------------
-- Walk state variables
--
-- These are continuously updated on each update
----------------------------------------------------------
-- Save the velocity between update cycles
local velCurrent = vector.new{0, 0, 0}

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local ankleShift = vector.new{0, 0}
local kneeShift  = 0
local hipShift   = vector.new{0, 0}

-- Still have an initial step for now
local initial_step, iStep

--------------------
-- Local Functions
--------------------

-- Get and massage gyro readings
local function get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local body_yaw
  if supportLeg == 0 then  -- Left support
    body_yaw = uLeft[3]  - uTorsoActual[3]
  else
    body_yaw = uRight[3] - uTorsoActual[3]
  end
  -- Ankle stabilization using gyro feedback
  --local imu_roll0, imu_pitch0, imu_yaw0 = unpack(Body.get_sensor_imu())
  --math.sin(imuPitch)*bodyHeight, -math.sin(imuRoll)*bodyHeight
  local gyro_roll0, gyro_pitch0, gyro_yaw0 = unpack(Body.get_sensor_gyro())
  -- Get effective gyro angle considering body yaw offset
  -- Rotate the Roll and pitch about the intended body yaw
  local gyro_roll  = gyro_roll0  * math.cos(body_yaw) - gyro_pitch0 * math.sin(body_yaw)
  local gyro_pitch = gyro_pitch0 * math.cos(body_yaw) - gyro_roll0  * math.sin(body_yaw)

  -- Give these parameters
  return gyro_roll, gyro_pitch, gyro_yaw0
end

local function update_velocity(velCurrent)
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
  return velCurrent + velDiff
end

---------------------------
-- Handle walk requests --
---------------------------
--[[
local walk_requests = {}
-- Setting velocity
walk_requests.some_request = function()
  print('some request')
end
--]]

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
  mcm.set_walk_vel(velCurrent)

  -- Make our ZMP solver
  zmp_solver = libZMP.new_solver({
    ['tStep'] = Config.walk.tStep,
    ['tZMP']  = Config.walk.tZMP,
    ['start_phase']  = Config.walk.phSingle[1],
    ['finish_phase'] = Config.walk.phSingle[2],
  })

  --Read stored feet and torso poses 
  local uTorso0 = mcm.get_poses_uTorso()  
  local uLeft = mcm.get_poses_uLeft()
  local uRight = mcm.get_poses_uRight()

  uTorso_now, uTorso_next = uTorso0, uTorso0
  uLeft_now,  uLeft_next  = uLeft,  uLeft
  uRight_now, uRight_next = uRight, uRight
  
  -- Compute initial zmp from these foot positions
  -- initial support is midpoint between feet: uTorso
  -- This is because we assume we are standing prior 
  -- to entering this state
  zmp_solver:compute( uTorso0, uTorso_now, uTorso_next )
  t_last_step = Body.get_time()

  -- Initialize the step index
  iStep = 1
  -- Initial step modification counter
  initial_step = 2

  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
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
  --[[
  local event, has_more
  repeat
    event, has_more = evts:receive(true)
    if type(event)=='string' then
      local request = walk_requests[event]
      --print( util.color('Walk Event:','green'),event)
      if request then request() end
    end
  until not has_more
  --]]
  ------------------------------------------

  -- SJ: Variable tStep support for walkkick
  -- Grab the phase of the current step
  local ph = (t-t_last_step)/zmp_solver.tStep
  if ph>1 then
    --Should we stop now?
    local stoprequest = mcm.get_walk_stoprequest()
    print(stoprequest)
    if stoprequest>0 then
      return"done"
    end
    -- TODO: reset the tStep, if variable
    -- can get it from mcm?
    -- Wrap around the phase
    ph = ph % 1
    -- Advance the steps to find the next desired torso position
    -- Increment the step index
    iStep = iStep + 1
    -- supportLeg: 0 for left support, 1 for right support
    supportLeg = iStep % 2
    -- Update the velocity via a filter
    velCurrent = update_velocity(velCurrent)
      -- If our first step(s), then use zero velocity
    if initial_step>0 then --Can be 2,1
      velCurrent = vector.new{0,0,0}
      initial_step = initial_step-1
    end
    -- Save the previous desired positions as the current desired positions
    -- TODO: Some feedback could get the uLeft/uRight/uTorso perfectly correct
    uLeft_now, uRight_now, uTorso_now = uLeft_next, uRight_next, uTorso_next
    -- Calculate the next step
    local uSupport
    -- TODO: We can queue steps, rather than use on the fly calculations...
    -- That could be sent to the walk engine as via points...?
    uSupport, uLeft_next, uRight_next = 
      moveleg.calculate_next_step( uLeft_now, uRight_now, supportLeg, velCurrent )

    if initial_step>0 then
      uLeft_next = uLeft_now;
      uRight_next = uRight_now;
    end

    -- This is the next desired torso position    
    uTorso_next = moveleg.calculate_next_torso( uLeft_next, uRight_next, supportLeg, 0.5 )
    if initial_step>0 then --support should be closer to center for initial step
      --1 for no swing, 0 for full swing
      uSupport = util.se2_interpolate(0.3,uSupport,uTorso_next)      
    end
    -- Compute coefficients for this step to 
    -- guide the legs and torso through the phase
    zmp_solver:compute( uSupport, uTorso_now, uTorso_next )
    -- Update t_last_step
    t_last_step = Body.get_time()
    -- Save some step-by-step data to shared memory
    mcm.set_status_velocity(velCurrent)
    mcm.set_support_uLeft_now(  uLeft_now )
    mcm.set_support_uRight_now( uRight_now )
    mcm.set_support_uTorso_now( uTorso_now )
    mcm.set_support_uLeft_next(  uLeft_next )
    mcm.set_support_uRight_next( uRight_next )
    mcm.set_support_uTorso_next( uTorso_next )

    -- Print a debugging message
    print( util.color('Walk velocity','blue'), string.format("%g, %g, %g",unpack(velCurrent)) )
  end
  -- Where does zmp think the torso should be?
  local uTorso = zmp_solver:get_com(ph)
  -- Where does zmp think the swing foot should be?
  local xFoot, zFoot, phSingle = zmp_solver:get_foot(ph)  
  --local xFoot, zFoot, phSingle = zmp_solver:get_foot_square(ph)  

  --Don't lift foot at initial step
  if initial_step>0 then
    zFoot = 0
  end

  -- Begin to solve for our leg positions
  local pLLeg = vector.new{0, footY,0, 0,0,0}
  local pRLeg = vector.new{0,-footY,0, 0,0,0}
  local uLeft, uRight
  if supportLeg == 0 then
    -- Left support
    uLeft = uLeft_now --uSupport?
    uRight   = util.se2_interpolate(xFoot, uRight_now, uRight_next)
    pRLeg[3] = stepHeight*zFoot
  else
    -- Right support
    uRight = uRight_now --uSupport?
    uLeft = util.se2_interpolate(xFoot, uLeft_now, uLeft_next)
    pLLeg[3] = stepHeight*zFoot
  end

  -- Adjust the yaw angle of the center of mass
  --com[3] = .5*(uLeft[3] + uRight[3])
  -- Linear speed turning
  uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2

  -- The reference frame is from the right foot, so reframe the torso
  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  -- Grab gyro feedback for these joint angles
  local r,p,y = get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )

  local delta_legs
  delta_legs, ankleShift, kneeShift, hipShift = moveleg.get_leg_compensation(
      supportLeg,phSingle,{r,p,y}, ankleShift, kneeShift, hipShift, initial_step)

  -- Set the current torso and feet transforms
  local pTorso = vector.new({
    uTorsoActual[1], uTorsoActual[2], Config.walk.bodyHeight,0,bodyTilt,uTorsoActual[3]})
  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3]
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3]
  
  moveleg.set_leg_positions(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)  
  
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
  mcm.set_poses_uLeft(uLeft_next)
  mcm.set_poses_uRight(uRight_next)
  mcm.set_poses_uTorso(uTorso_next)
  print(walk._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return walk
