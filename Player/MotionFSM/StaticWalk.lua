--New staticwalk


local walk = {}
walk._NAME = ...

local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local moveleg = require'moveleg'
require'mcm'

-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber('Walk',true)

-- Keep track of important times
local t_entry, t_update, t_last_step

-- Stance parameters
local bodyTilt = Config.walk.bodyTilt or 0
local torsoX   = Config.walk.torsoX
local footY    = Config.walk.footY

--Gait parameters
local stepHeight  = Config.walk.stepHeight
local tStep = Config.walk.tStep
---------------------------------------------------------
-- Walk state variables
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

  --Read stored feet and torso poses 
  local uTorso0 = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  uTorso_now, uTorso_next = uTorso0, uTorso0
  uLeft_now,  uLeft_next  = uLeft,  uLeft
  uRight_now, uRight_next = uRight, uRight

  t_last_step = Body.get_time()-tStep --for starting the next step right now

  iStep = 1   -- Initialize the step index  
  initial_step = 0 -- We don't need this for static walk
  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

  --SJ: walk events are simply handled via SHM 
  local stoprequest = mcm.get_walk_stoprequest()

  local ph = (t-t_last_step)/tStep
  if ph>1 then  
    if stoprequest>0 then return"done" end --Should we stop now?
    ph = ph % 1
    iStep = iStep + 1  -- Increment the step index    
    supportLeg = iStep % 2 -- supportLeg: 0 for left support, 1 for right support
    velCurrent = moveleg.update_velocity(velCurrent)     -- Update the velocity via a filter
      -- If our first step(s), then use zero velocity
    if initial_step>0 then --Can be 2,1
      velCurrent = vector.new{0,0,0}
      initial_step = initial_step-1
    end
    uLeft_now, uRight_now, uTorso_now = uLeft_next, uRight_next, uTorso_next
    uSupport, uLeft_next, uRight_next = 
      moveleg.calculate_next_step( uLeft_now, uRight_now, supportLeg, velCurrent )
    if initial_step>0 then uLeft_next = uLeft_now;uRight_next = uRight_now;end

    -- This is the next desired torso position    
    uTorso_next = moveleg.calculate_next_torso( uLeft_next, uRight_next, supportLeg, 0.5 )
    if initial_step>0 then --support should be closer to center for initial step
      --1 for no swing, 0 for full swing
      uSupport = util.se2_interpolate(0.3,uSupport,uTorso_next)      
    end
  
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

  ------------------------------------------------------------
  --Static COM movement
  local uTorso
  phSingle1 = 0.3
  phSingle2 = 0.7
  if ph<phSingle1 then
    ph2 = ph/0.3;
    uTorso = util.se2_interpolate( ph2*(2-ph2)  ,uTorso_now, uSupport)
  elseif ph<phSingle2 then
    uTorso = uSupport
  else
    ph2 = (1-ph)/(1-phSingle2);
    uTorso = util.se2_interpolate( ph2*(2-ph2) ,uTorso_next, uSupport)
  end  
  uTorso[3] = ph*(uLeft_next[3]+uRight_next[3])/2 + (1-ph)*(uLeft_now[3]+uRight_now[3])/2
  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  --------------------------------------------------------------------
  
  phFootSingle1 = 0.3
  phFootSingle2 = 0.7
  local xFoot, zFoot, phSingle = moveleg.get_foot_square(
      ph,phFootSingle1,phFootSingle2)  
  --Don't lift foot at initial step
  if initial_step>0 then zFoot = 0  end

  -- Begin to solve for our leg positions  
  local uLeft, uRight, zLeft, zRight
  if supportLeg == 0 then
    -- Left support
    uLeft = uLeft_now 
    uRight = util.se2_interpolate(xFoot, uRight_now, uRight_next)
    zLeft = 0
    zRight =  stepHeight*zFoot
  else
    -- Right support
    uRight = uRight_now 
    uLeft = util.se2_interpolate(xFoot, uLeft_now, uLeft_next)
    zLeft =  stepHeight*zFoot    
    zRight = 0
  end

  -- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local delta_legs
  delta_legs, ankleShift, kneeShift, hipShift = moveleg.get_leg_compensation(
      supportLeg,phSingle,gyro_rpy, ankleShift, kneeShift, hipShift, initial_step)

  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], Config.walk.bodyHeight,
        0,bodyTilt,uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
    
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
  mcm.set_status_uLeft(uLeft_next)
  mcm.set_status_uRight(uRight_next)
  mcm.set_status_uTorso(uTorso_next)
  print(walk._NAME..' Exit')  
end

return walk
