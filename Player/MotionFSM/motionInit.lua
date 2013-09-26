local state = {}
state._NAME = ...

--motionInit: initialize legs to correct position


require'mcm'
local Body       = require'Body'
local K          = Body.Kinematics
local util       = require'util'
local vector     = require'vector'
local timeout    = 20.0
local t_readings = 0.20
local t_settle   = 0.10
-- Declare local so as not to pollute the global namespace
-- This is 5.1 and 5.2 compatible
-- NOTE: http://www.luafaq.org/#T1.37.1
local t_entry, t_update, t_finish


local pLLeg_desired, pRLeg_desired, pTorso_desired, pTorso
-- Desired Waist position and limits
local qWaist_desired, dqWaistLimit
local dpMaxDelta = Config.stance.dpLimitStance


  -- Set the default waist
local qWaist_desired = Config.stance.qWaist
local dqWaistLimit   = Config.stance.dqWaistLimit

  -- Set the desired legs
local pLLeg_desired = Config.stance.pLLeg
local pRLeg_desired = Config.stance.pRLeg
  -- Desired torso
local  pTorso_desired = Config.stance.pTorso



function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t_entry
  
  --SJ: Now we only use commanded positions
  --As the actual values are read at motionIdle state
  local qLLeg = Body.get_lleg_command_position()
  local qRLeg = Body.get_rleg_command_position()
    
  -- How far away from the torso are the legs currently?
  local dpLLeg = K.torso_lleg(qLLeg)
  local dpRLeg = K.torso_rleg(qRLeg)
  
  local pTorsoL = pLLeg_desired + dpLLeg
  local pTorsoR = pRLeg_desired + dpRLeg
  pTorso = (pTorsoL+pTorsoR)/2
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
   
  -- Zero the waist  
  local qWaist = Body.get_waist_command_position()
  local qWaist_approach, doneWaist = 
    util.approachTol( qWaist, qWaist_desired, dqWaistLimit, dt )
  Body.set_waist_command_position(qWaist_approach)

  -- Ensure that we do not move motors too quickly
  local pTorso_approach, doneTorso = 
    util.approachTol( pTorso, pTorso_desired, dpMaxDelta, dt )
  -- If not yet within tolerance, then update the last known finish time

  if not doneTorso or not doneWaist then t_finish = t end --do we need this?
  
  -- Command the body
  -- TODO: Should we approach these positions?
  local qLegs = Kinematics.inverse_legs( pLLeg_desired, pRLeg_desired, pTorso_approach, 0 )

  if Config.stance.enable_legs then
    Body.set_lleg_command_position( qLegs )  
    -- Once in tolerance, let the robot settle
    if t-t_finish>t_settle then return'done' end
  else
    return true
  end  
end

function state.exit()
  print(state._NAME..' Exit.  Time elapsed:',t_finish-t_entry )
  -- now on feet
  mcm.set_walk_bipedal(1)
  -- Update current pose for use by the camera
  mcm.set_camera_bodyHeight(pTorso[3])
  mcm.set_camera_bodyTilt(pTorso[5])
end

return state