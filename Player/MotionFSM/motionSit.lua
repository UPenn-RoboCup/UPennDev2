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


-- Keep track of important times
local t_entry, t_update, t_last_step

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local uTorso0, uTorsoComp, uTorsoKneel
local stage




local velBodyTilt = 5*math.pi/180
local velWaist = {3*Body.DEG_TO_RAD, 0}
local velTorso = {0.03,0.03,0}
local velHeight = 0.03

local velBodyTilt2 = 1*math.pi/180
  
local bodyTiltSit = -10*math.pi/180
local bodyHeightSit = 0.60; 

local uTorsoKneel = {0.15,0,0}
local bodyTiltKneel = Config.walk.bodyTilt
--local qWaistTarget = {45*Body.DEG_TO_RAD,0,0}
local qWaistTarget = {0*Body.DEG_TO_RAD,0,0}

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
  
  uTorso0 = mcm.get_status_uTorso()  
  uTorsoComp = {0,0,0}
  stage = 1
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
    local bodyTilt,tiltDone = util.approachTol(bodyTiltNow, bodyTiltSit, velBodyTilt, t_diff )
    local bodyHeight,heightDone = util.approachTol(bodyHeightNow, bodyHeightSit, velHeight, t_diff )
    mcm.set_stance_bodyTilt(bodyTilt)
    mcm.set_stance_bodyHeight(bodyHeight)
 
     --TODO: it may be better to kill gyro feedback while sitting/standing
    local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorso, supportLeg )
    delta_legs, angleShift = moveleg.get_leg_compensation(2,0,gyro_rpy, angleShift)
    moveleg.set_leg_positions(uTorso,uLeft,uRight, 0,0, delta_legs)

    if tiltDone and heightDone then stage = stage+1  end
  elseif stage==2 then
    mcm.set_status_iskneeling(1)
    local bodyTilt, tiltDone = util.approachTol( bodyTiltNow, bodyTiltKneel, velBodyTilt2, t_diff )
    mcm.set_stance_bodyTilt(bodyTilt)    
    uTorsoComp,torsoDone = util.approachTol(uTorsoComp,uTorsoKneel,velTorso,t_diff)
    mcm.set_status_uTorso(util.pose_global(uTorsoComp,uTorso0))  
    moveleg.set_leg_positions_kneel(t_diff)    
    if tiltDone and torsoDone then stage = stage+1 end
  elseif stage==3 then

    local qWaist = Body.get_waist_command_position()
    local qWaist, waistDone = util.approachTol( qWaist, qWaistTarget, velWaist, t_diff )
    Body.set_waist_command_position(qWaist)
  end
 
end 

function state.exit()
  
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state
