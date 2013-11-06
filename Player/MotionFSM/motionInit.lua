local state = {}
state._NAME = ...

--motionInit: initialize legs to correct position


require'mcm'
require'hcm'
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

  -- Set the desired leg and torso poses
local pLLeg_desired = Config.stance.pLLeg
local pRLeg_desired = Config.stance.pRLeg
local pTorso_desired = Config.stance.pTorso
  -- Set the desired waist
local qWaist_desired = Config.stance.qWaist

  -- Set movement speed limits
local dpMaxDelta = Config.stance.dpLimitStance
local dqWaistLimit   = Config.stance.dqWaistLimit
local dqLegLimit = Config.stance.dqLegLimit


--local dqLegLimit = vector.new{10,10,10,10,10,10}*Body.DEG_TO_RAD

dpMaxDelta = vector.new{.04, .03, .03, .4, .1, .4}



local pTorso, qLLeg, qRLeg

stage = 1

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t_entry
  
  --SJ: Now we only use commanded positions
  --As the actual values are read at motionIdle state
  qLLeg = Body.get_lleg_command_position()
  qRLeg = Body.get_rleg_command_position()

  -- How far away from the torso are the legs currently?
  local dpLLeg = K.torso_lleg(qLLeg)
  local dpRLeg = K.torso_rleg(qRLeg)
  
  local pTorsoL = pLLeg_desired + dpLLeg
  local pTorsoR = pRLeg_desired + dpRLeg
  pTorso = (pTorsoL+pTorsoR)/2

  mcm.set_stance_bodyHeight(pTorso[3])
  mcm.set_stance_bodyTilt(pTorso[5])

  stage = 1
  if not IS_WEBOTS then
    for i=1,10 do
      Body.set_waist_command_velocity({500,500})
      unix.usleep(1e6*0.01);
      Body.set_lleg_command_velocity({500,500,500,500,500,500})
      unix.usleep(1e6*0.01);
      Body.set_rleg_command_velocity({500,500,500,500,500,500})
      unix.usleep(1e6*0.01);  
      Body.set_rleg_command_acceleration({50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
      Body.set_lleg_command_acceleration({50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
    end
  end

  Body.request_lleg_position()
  Body.request_rleg_position()

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
  local qLegsTarget = Kinematics.inverse_legs( pLLeg_desired, pRLeg_desired, pTorso_approach, 0 )
  local qLLegTarget = vector.slice(qLegsTarget,1,6)
  local qRLegTarget = vector.slice(qLegsTarget,7,12)

  qLLegMove,doneL = util.approachTol(qLLeg,qLLegTarget, dqLegLimit, dt )
  qRLegMove,doneR = util.approachTol(qRLeg,qRLegTarget, dqLegLimit, dt )

  Body.set_lleg_command_position(qLLegMove)
  Body.set_rleg_command_position(qRLegMove)

  local qLLegActual = Body.get_lleg_position()
  local qRLegActual = Body.get_rleg_position()
  local qLLegCommand = Body.get_lleg_command_position()
  local qRLegCommand = Body.get_rleg_command_position()

  Body.request_lleg_position()
  Body.request_rleg_position()

  local err = 0;
  for i=1,4 do --hack: skip ankle angles for now
    err = err + math.abs(qLLegActual[i]- qLLegCommand[i])
    err = err + math.abs(qRLegActual[i]- qRLegCommand[i])
  end
  --print("err: ",err, doneL,doneR)

  local err_th = 1*Body.DEG_TO_RAD
  
  if (err<err_th or IS_WEBOTS) and t-t_finish>t_settle and doneL and doneR then return'done' end        
end

function state.exit()
  print(state._NAME..' Exit.  Time elapsed:',t_finish-t_entry )
  -- now on feet
  mcm.set_walk_bipedal(1)
  -- Update current pose for use by the camera
  mcm.set_stance_bodyHeight(pTorso[3])
  mcm.set_stance_bodyTilt(pTorso[5])

  local footY    = Config.walk.footY
  local supportX = Config.walk.supportX

  --Generate current 2D pose for feet and torso
  local uTorso = vector.new({supportX, 0, 0})
  local uLeft  = util.pose_global(vector.new({-supportX, footY, 0}),uTorso)
  local uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso)    
  mcm.set_status_uLeft(uLeft)
  mcm.set_status_uRight(uRight)
  mcm.set_status_uTorso(uTorso)
  mcm.set_status_uTorsoVel(vector.new{0,0,0})

  mcm.set_stance_bodyHeight(Config.walk.bodyHeight)
  mcm.set_stance_bodyHeightTarget(Config.walk.bodyHeight)  

  mcm.set_stance_uTorsoComp({0,0})
  mcm.set_status_iskneeling(0)
  
  local pg = Config.walk.leg_p_gain or 64

  if not IS_WEBOTS then
    for i=1,10 do
      Body.set_lleg_command_velocity({17000,17000,17000,17000,17000,17000})
      unix.usleep(1e6*0.01);

      Body.set_rleg_command_velocity({17000,17000,17000,17000,17000,17000})
      unix.usleep(1e6*0.01);  

      Body.set_rleg_command_acceleration({200,200,200,200,200,200})
      unix.usleep(1e6*0.01);

      Body.set_lleg_command_acceleration({200,200,200,200,200,200})
      unix.usleep(1e6*0.01);

      Body.set_rleg_position_p({pg,pg,pg,pg,pg,pg})
      unix.usleep(1e6*0.01);

      Body.set_lleg_position_p({pg,pg,pg,pg,pg,pg})
      unix.usleep(1e6*0.01);
    end
  end
  mcm.set_walk_ismoving(0) --We are stopped
end

return state
