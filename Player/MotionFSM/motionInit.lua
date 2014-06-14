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
local t_entry, t_update, t_finish

  -- Set the desired leg and torso poses
local pLLeg_desired = vector.new{-Config.walk.supportX,  Config.walk.footY, 0, 0,0,0}
local pRLeg_desired = vector.new{-Config.walk.supportX,  -Config.walk.footY, 0, 0,0,0}
local pTorso_desired = vector.new{-Config.walk.torsoX, 0, Config.walk.bodyHeight, 0,Config.walk.bodyTilt,0}


-- Set the desired waist
local qWaist_desired = Config.stance.qWaist

-- Set movement speed limits
local dpMaxDelta = Config.stance.dpLimitStance
local dqWaistLimit = Config.stance.dqWaistLimit
local dqLegLimit = Config.stance.dqLegLimit

local pTorso, qLLeg, qRLeg

local stage = 1

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t_entry

  --SJ: Now we only use commanded positions
  --As the actual values are read at motionIdle state
  qLLeg = Body.get_lleg_position()
  qRLeg = Body.get_rleg_position()

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
    print('INIT setting params')
    for i=1,10 do

      Body.set_head_command_velocity({500,500})
      unix.usleep(1e6*0.01);


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

  if IS_WEBOTS then
    print('hi')
    return'done'
  end

  -- Zero the waist

  local qWaist = Body.get_waist_position()
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


  local legBias = vector.new(mcm.get_leg_bias())
  qLegsTarget = vector.new(qLegsTarget) + legBias

  local qLLegTarget = vector.slice(qLegsTarget,1,6)
  local qRLegTarget = vector.slice(qLegsTarget,7,12)

  qLLegMove,doneL = util.approachTol(qLLeg,qLLegTarget, dqLegLimit, dt )
  qRLegMove,doneR = util.approachTol(qRLeg,qRLegTarget, dqLegLimit, dt )

  Body.set_lleg_command_position(qLLegMove)
  Body.set_rleg_command_position(qRLegMove)

  local qLLegActual = Body.get_lleg_position()
  local qRLegActual = Body.get_rleg_position()
  local qWaistActual = Body.get_waist_position()

  local qLLegCommand = Body.get_lleg_command_position()
  local qRLegCommand = Body.get_rleg_command_position()
  local qWaistCommand = Body.get_waist_command_position()


  local err = 0;
  for i=1,4 do --hack: skip ankle angles for now
    err = err + math.abs(qLLegActual[i]- qLLegCommand[i])
    err = err + math.abs(qRLegActual[i]- qRLegCommand[i])
  end
  err = err + math.abs(qWaistActual[1]- qWaistCommand[1])
  err = err + math.abs(qWaistActual[2]- qWaistCommand[2])

  --print("err: ",err, doneL,doneR)

  local err_th = 1*DEG_TO_RAD

--  if (err<err_th or IS_WEBOTS) and t-t_finish>t_settle and doneL and doneR then return'done' end
  if (err<err_th or IS_WEBOTS) then return'done' end
end

function state.exit()
  print(state._NAME..' Exit.  Time elapsed:',t_finish-t_entry )

  -- now on feet
  mcm.set_walk_bipedal(1)

  -- Update current pose
  mcm.set_stance_bodyHeight(pTorso[3])
  mcm.set_stance_bodyTilt(pTorso[5])
  hcm.set_motion_bodyHeightTarget(pTorso[3])


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
  local ag = Config.walk.ankle_p_gain or 64

  if not IS_WEBOTS then
    for i=1,10 do

--[[
      Body.set_lleg_command_velocity({17000,17000,17000,17000,17000,17000})
      unix.usleep(1e6*0.01);

      Body.set_rleg_command_velocity({17000,17000,17000,17000,17000,17000})
      unix.usleep(1e6*0.01);

      Body.set_rleg_command_acceleration({200,200,200,200,200,200})
      unix.usleep(1e6*0.01);

      Body.set_lleg_command_acceleration({200,200,200,200,200,200})
      unix.usleep(1e6*0.01);
--]]


--set to zero = max value





      Body.set_head_command_velocity({1000,1000})
      unix.usleep(1e6*0.01);


      Body.set_waist_command_velocity({0,0})
      unix.usleep(1e6*0.01);

      Body.set_lleg_command_velocity({0,0,0,0,0,0})
      unix.usleep(1e6*0.01);

      Body.set_rleg_command_velocity({0,0,0,0,0,0})
      unix.usleep(1e6*0.01);

      Body.set_rleg_command_acceleration({0,0,0,0,0,0})
      unix.usleep(1e6*0.01);

      Body.set_lleg_command_acceleration({0,0,0,0,0,0})
      unix.usleep(1e6*0.01);


--SJ: this somehow locks down head movement!!!!!!!!
--[[
      Body.set_rleg_position_p({pg,pg,pg,pg,pg,ag})
      unix.usleep(1e6*0.01);

      Body.set_lleg_position_p({pg,pg,pg,pg,pg,ag})
      unix.usleep(1e6*0.01);
--]]
    end
  end
  mcm.set_walk_ismoving(0) --We are stopped
end

return state
