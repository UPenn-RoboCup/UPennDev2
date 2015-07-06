--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local t_entry, t_update, t_finish, t_last_debug
local timeout = 15.0

-- Goal position is arm Init, with hands in front, ready to manipulate
local qLArmTarget, qRArmTarget
local last_error


local function setArmJoints(qLArmTarget,qRArmTarget, dt,dqArmLim, absolute)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()  

  local qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqArmLim, dt ,nil,absolute)  
  local qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqArmLim, dt ,nil,absolute)

  if not absolute then  
    for i=1,7 do
      local qL_increment = util.mod_angle(qL_approach[i]-qLArm[i])
      local qR_increment = util.mod_angle(qR_approach[i]-qRArm[i])
      qL_approach[i] = qLArm[i] + qL_increment
      qR_approach[i] = qRArm[i] + qR_increment
    end
  end

  Body.set_larm_command_position( qL_approach )
  Body.set_rarm_command_position( qR_approach )
  if doneL2 and doneR2 then return 1 end
end


--SJ: now SLOWLY move joint one by one
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  -- Close rgrip
  Body.set_rgrip_command_torque({-10,-10,10})
  -- Open rgrip
  Body.set_rgrip_command_torque({10,10,-10})
  -- No torque rgrip
  Body.set_rgrip_command_torque({0,0,0})

  qLArmTarget = Body.get_inverse_larm(
    vector.zeros(7),
    Config.arm.trLArm0,
    Config.arm.ShoulderYaw0[1],
    mcm.get_stance_bodyTilt(),{0,0},true)

  qRArmTarget = Body.get_inverse_rarm(
    vector.zeros(7),
    Config.arm.trRArm0,
    Config.arm.ShoulderYaw0[2],
    mcm.get_stance_bodyTilt(),{0,0},true)

print("L:",qLArmTarget[5]*RAD_TO_DEG)
print("R:",qRArmTarget[5]*RAD_TO_DEG)

  qLArmTarget = {110*DEG_TO_RAD,0,0,   -150*DEG_TO_RAD, 90*DEG_TO_RAD,40*DEG_TO_RAD,-90*DEG_TO_RAD}
  qRArmTarget = {110*DEG_TO_RAD,0,0,   -150*DEG_TO_RAD, -90*DEG_TO_RAD,-40*DEG_TO_RAD,90*DEG_TO_RAD}


  qLArmTarget = vector.new({110,0,10,-155,90,45,-76})*DEG_TO_RAD
  qRArmTarget = vector.new({110,0,-10,-160,-90,-40,76})*DEG_TO_RAD


  t_last_debug=t_entry
  last_error = 999
  stage = 1
  t_stage_start = t_entry
  t_stage = 1
end


local shoulderRollInit = 10*math.pi/180

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
    
  local ret
  local qLArmTargetC, qRArmTargetC = util.shallow_copy(qLArm),util.shallow_copy(qRArm)
    
  if stage==1 then --Straighten wrist roll and second wrist yaw       
    qLArmTargetC[6],qRArmTargetC[6] = 0,0
    qLArmTargetC[7],qRArmTargetC[7] = qLArmTarget[7],qRArmTarget[7]
    t_stage = 1.0

  elseif stage==2 then  --Straighten first wrist yaw, straighten shouldr yaw, widen shoulder
    qLArmTargetC[2],qRArmTargetC[2] = shoulderRollInit, -shoulderRollInit
    qLArmTargetC[3],qRArmTargetC[3] = qLArmTarget[3],qRArmTarget[3]
    qLArmTargetC[5],qRArmTargetC[5] = qLArmTarget[5],qRArmTarget[5]
    qLArmTargetC[6],qRArmTargetC[6] = 0,0
    qLArmTargetC[7],qRArmTargetC[7] = qLArmTarget[7],qRArmTarget[7]
    t_stage = 1.0

  elseif stage==3 then --straighten shoulder yaw, widen shoulder
    qLArmTargetC,qRArmTargetC = 
	util.shallow_copy(qLArmTarget),
	util.shallow_copy(qRArmTarget)
    qLArmTargetC[2],qRArmTargetC[2] = shoulderRollInit, -shoulderRollInit
    t_stage = 2.0

  elseif stage==4 then
    qLArmTargetC,qRArmTargetC = qLArmTarget,qRArmTarget
    t_stage = 1.0
  elseif stage==5 then
    return "done"
  end


  local dqArmLim = vector.new(util.shallow_copy(Config.arm.vel_angular_limit_init))
  if IS_WEBOTS then dqArmLim = dqArmLim*2 end

  local ret = setArmJoints(qLArmTargetC,qRArmTargetC,dt,dqArmLim,true)

  --SJ: initial waist position can be either +2 or -358
  --so we should be careful using it for jacobian

  Body.set_safe_waist_command_position({0,0})
--  local ret = setArmJoints(qLArmTargetC,qRArmTargetC,dt,dqArmLim,false) --should use absolute position (for jacobian)
  local qLArmActual = Body.get_larm_position()
  local qRArmActual = Body.get_rarm_position()
  local qWaistActual = Body.get_waist_position()
  local qLArmCommand = Body.get_larm_command_position()
  local qRArmCommand = Body.get_rarm_command_position()
  local qWaistCommand = Body.get_waist_command_position()

  local err=0
  err=err+math.abs(qWaistActual[1]-qWaistCommand[1])
  err=err+math.abs(qWaistActual[2]-qWaistCommand[2])
  for i=1,7 do
    err=err+math.abs(qLArmActual[i]-qLArmCommand[i])
    err=err+math.abs(qRArmActual[i]-qRArmCommand[i])
  end

--
  if ret==1 and Config.arm_init_timeout and t-t_stage_start>t_stage then
    t_stage_start = t
    stage=stage+1
    last_error=err
    return
  end
--]]

  if t>t_last_debug+0.2 then
    t_last_debug=t
    if ret==1 and math.abs(last_error-err)<0.2*math.pi/180 then 
      stage = stage+1
      print("Total joint reading err:",err*180/math.pi)
    end
    last_error = err
  end    
end

function state.exit()

  --Our arm joint angles are (qLArmTarget, qRArmTarget)
  --but as the torso moves to compensate. the actual arm end positions are slightly different
  --we calculated the arm transforms (sans compensation) and stores them as current intiial conditions for planner

  local qWaist = Body.get_waist_command_position()
  local uTorsoComp = Body.get_torso_compensation(qLArmTarget,qRArmTarget,qWaist)  
  local vec_comp = vector.new({uTorsoComp[1],uTorsoComp[2],0, 0,0,0})
  local trLArmComp = Body.get_forward_larm(qLArmTarget)
  local trRArmComp = Body.get_forward_rarm(qRArmTarget)
  local trLArm = vector.new(trLArmComp) + vec_comp
  local trRArm = vector.new(trRArmComp) + vec_comp 
  mcm.set_arm_trlarm(trLArm)
  mcm.set_arm_trrarm(trRArm)
  mcm.set_arm_qlarmcomp(qLArmTarget)
  mcm.set_arm_qrarmcomp(qRArmTarget)
  mcm.set_stance_uTorsoComp(uTorsoComp)  


  mcm.set_status_arm_init(1) --arm idle and requires init


--SJ: now we store the COM offset for default arm posture
  local COMoffset = mcm.get_stance_COMoffset()
  mcm.set_stance_COMoffsetPose1(COMoffset)
  print("COMoffset:",unpack(COMoffset))
  --print("uTorsoComp",unpack(uTorsoComp))
  print(state._NAME..' Exit' )
end

return state

