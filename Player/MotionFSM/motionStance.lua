require'mcm'
local Config     = require'Config'
local Body       = require'Body'
local util       = require'util'
local K          = Body.Kinematics
local timeout    = 20.0
local t_readings = 0.20
local t_settle   = 0.10
-- Declare local so as not to pollute the global namespace
-- This is 5.1 and 5.2 compatible
-- NOTE: http://www.luafaq.org/#T1.37.1
local t_entry, t_update, t_finish
local pTorsoTarget, pLLeg, pRLeg, pTorso
local started = false

local state = {}
state._NAME = 'motionStance'

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t
  
  -- Grab the desired final torso and feet positions for stance
  pTorsoTarget = vector.new({-Config.walk.footX, 0, Config.walk.bodyHeight, 
  0,Config.walk.bodyTilt,0})
  pLLeg = vector.new({-Config.walk.supportX, Config.walk.footY, 0, 0,0,0})
  pRLeg = vector.new({-Config.walk.supportX, -Config.walk.footY, 0, 0,0,0})
  
  -- Instantly put the head and waist into position
  -- TODO: replace hardness
  -- TODO: abstract the number of waist joints
  --Body.set_head_hardness(.5)
  --Body.set_waist_hardness(1)
  Body.set_head_command_position({0,0})
  Body.set_waist_command_position({0,0})
  
  -- TODO: Set walk.active to false, using mcm
  
  -- TODO: Enqueue joint angle readings
  
  started = false

end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end
  
  -- Wait for the joint angle readings to return from every motor
  if t-t_entry<t_readings then return end
  
  -- Acquire the joint positions of our legs
  if not started then
    local qLLeg  = Body.get_lleg_position()
    local qRLeg  = Body.get_rleg_position()
    Body.set_lleg_command_position(qLLeg)
    Body.set_rleg_command_position(qRLeg)
    
    -- Solve for the current torso position
    local dpLLeg = K.torso_lleg(qLLeg)
    local dpRLeg = K.torso_rleg(qRLeg)
    local pTorsoL = pLLeg+dpLLeg
    local pTorsoR = pRLeg+dpRLeg
    pTorso = (pTorsoL+pTorsoR)/2
    -- TODO: Hardness
    --Body.set_lleg_hardness(hardnessLeg)
    --Body.set_rleg_hardness(hardnessLeg)
    started = true
  end
  
  -- Ensure that we do not move motors too quickly
  local dpDeltaMax = Config.stance.dpLimitStance * dt
  local dpTorso    = pTorsoTarget - pTorso  
  -- Tolerance check (Asumme within tolerance)
  local tol      = true
  local tolLimit = 1e-6
  for i,dpT in ipairs(dpTorso) do
    if math.abs(dpT) > tolLimit then
      tol = false
      dpTorso[i] = util.procFunc(dpT,0,dpDeltaMax[i])
    end
  end
  
  -- Set the desired torso
  pTorso = pTorso + dpTorso
  -- Command the body
  local q = Kinematics.inverse_legs( pLLeg, pRLeg, pTorso, 0 )
  Body.set_lleg_command_position( q )
  
  -- If not yet within tolerance, then update the last known finish time only
  if not tol then
    t_finish = t
    return
  end
  
  -- Once in tolerance, let the robot settle
  if t-t_finish>t_settle then return'done' end
  
end

function state.exit()
  print(state._NAME..' Exit.  Time elapsed:',t_finish-t_entry )
  
  -- now on feet
  mcm.set_walk_bipedal(1)
  
  -- Update current pose for use by the camera
  mcm.set_camera_bodyHeight(pTorso[3])
  mcm.set_camera_bodyTilt(pTorso[5])
  
  -- Reset the stance in the walk controller
  -- TODO: Send on a walk channel or something?
  --walk.stance_reset()
end

return state