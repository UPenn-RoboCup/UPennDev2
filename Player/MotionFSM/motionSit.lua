--Stance state is basically a Walk controller
--Without any torso or feet update
--We share the leg joint generation / balancing code 
--with walk controllers

local state = {}
state._NAME = ...

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

local bodyHeight_next

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local uTorso0, uTorsoComp, uTorsoKneel
local stage

local velBodyTilt = 2*math.pi/180

---------------------------
-- State machine methods --
---------------------------
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  mcm.set_walk_bipedal(1)  
  bodyHeight_next = mcm.get_stance_bodyHeight()  

  stage = 1

  local uTorso = mcm.get_status_uTorso()  
  uTorso0 = uTorso
  uTorsoComp = {0,0,0}
  velWaist = {3*Body.DEG_TO_RAD, 0}
  
  uTorsoKneel = {0.10,0,0}
  qWaistTarget = {45*Body.DEG_TO_RAD,0,0}
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  
--Read stored feet and torso poses 
  local uTorso = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  --Adjust body height

  local bodyTiltNow = mcm.get_stance_bodyTilt()  
  local bodyHeightNow = mcm.get_stance_bodyHeight()  

  if stage==1 then
    local bodyTilt_target = -10*math.pi/180
    bodyHeight_target = 0.55; 
    

     local bodyTilt, tiltDone = util.approachTol( bodyTiltNow, 
      bodyTilt_target, velBodyTilt, t_diff )
    local bodyHeight, heightDone = util.approachTol( bodyHeightNow, 
      bodyHeight_target, Config.stance.dHeight, t_diff )

    mcm.set_stance_bodyTilt(bodyTilt)
    mcm.set_stance_bodyHeight(bodyHeight)

    local zLeft,zRight = 0,0
    supportLeg = 2; --Double support

    -- Grab gyro feedback for these joint angles
    local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )

    --TODO: it may be better to kill gyro feedback while sitting/standing
    delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,0,gyro_rpy, angleShift)

    moveleg.set_leg_positions(uTorso,uLeft,uRight, 0,0, delta_legs)
    if tiltDone and heightDone then stage = stage+1  end
  elseif stage==2 then
    local bodyTilt, tiltDone = util.approachTol( bodyTiltNow, 0, velBodyTilt, t_diff )
    mcm.set_stance_bodyTilt(bodyTilt)    
    uTorsoComp,torsoDone = util.approachTol(uTorsoComp,uTorsoKneel,{0.01,0,0},t_diff)
    moveleg.set_leg_positions_kneel(
      util.pose_global(uTorsoComp,uTorso0),uLeft,uRight, t_diff)
    if tiltDone and torsoDone then stage = stage+1 end
  elseif stage==3 then
    local qWaist = Body.get_waist_command_position()
    local qWaist, waistDone = util.approachTol( qWaist, qWaistTarget, velWaist, t_diff )

    Body.set_waist_command_position(qWaist)
  end
 

  

 
end -- walk.update

function state.exit()
  
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state