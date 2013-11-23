local movearm={}
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'hcm'

-- Angular velocity limit
local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit




local lShoulderYaw = -45*Body.DEG_TO_RAD;
local rShoulderYaw = 45*Body.DEG_TO_RAD;

function movearm.setArmJoints(qLArmTarget,qRArmTarget, dt,dqArmLim)
  dqArmLim = dqArmLim or Config.arm.slow_limit

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()  
   local qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqArmLim, dt )  
  local qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqArmLim, dt )

  --Dynamixel is STUPID so we should manually check for the direction
  for i=1,7 do
    local qL_increment = util.mod_angle(qL_approach[i]-qLArm[i])
    local qR_increment = util.mod_angle(qR_approach[i]-qRArm[i])
    qL_approach[i] = qLArm[i] + qL_increment
    qR_approach[i] = qRArm[i] + qR_increment
  end

  Body.set_larm_command_position( qL_approach )
  Body.set_rarm_command_position( qR_approach )
  if doneL2 and doneR2 then return 1 end
end

function movearm.setWristPosition(trLWristTarget, trRWristTarget, dt)
  
  --Interpolate in 6D space 
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLWrist = Body.get_forward_lwrist(qLArm);
  local trRWrist = Body.get_forward_rwrist(qRArm);
  local trLWristApproach, doneL = util.approachTol(trLWrist, trLWristTarget, dpArmMax, dt )
  local trRWristApproach, doneR = util.approachTol(trRWrist, trRWristTarget, dpArmMax, dt )

  -- Get desired angles from current angles and target transform
  local qL_desired = Body.get_inverse_lwrist(qLArm,trLWristApproach, qLArm[3])
  local qR_desired = Body.get_inverse_rwrist(qRArm,trRWristApproach, qRArm[3])  
  if qL_desired and qR_desired then
    local done = movearm.setArmJoints(qL_desired, qR_desired, dt)
    if done then return 1
    else return 0 end
  else    
    print("ERROR WITH WRIST IK")
    return -1;
  end  
end

function movearm.getDoorHandlePosition(pos_offset,knob_roll,door_yaw)
  local door_model = hcm.get_door_model()  
  local hinge_pos = vector.slice(door_model,1,3) + vector.new(pos_offset)  
  local door_r = door_model[4]
  local grip_offset_x = door_model[5]
  local knob_offset_y = door_model[6]

  local hand_rpy = Config.armfsm.dooropen.rhand_rpy
  local hand_yaw = door_yaw
  if door_yaw>10*Body.DEG_TO_RAD then
    hand_yaw = door_yaw-(door_yaw-10*Body.DEG_TO_RAD)*2.5 
  end

  local trHandle = T.eye()
    * T.trans(hinge_pos[1],hinge_pos[2],hinge_pos[3])    
    * T.rotZ(door_yaw)
    * T.trans(grip_offset_x, door_r+knob_offset_y, 0)     
    * T.rotX(knob_roll)
    * T.trans(0,-knob_offset_y, 0) 
    * T.rotX(-knob_roll)
    * T.rotZ(hand_yaw-door_yaw)
    * T.rotX(knob_roll)
    * T.transform6D(
      {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
  local trTarget = T.position6D(trHandle)
  return trTarget
end

--Push open only (use left arm chopsticks)
function movearm.getDoorLeftHandlePosition(pos_offset,knob_roll,door_yaw)
  local door_model = hcm.get_door_model()  
  local hinge_pos = vector.slice(door_model,1,3) + vector.new(pos_offset)  

  local door_r = door_model[4]
  local grip_offset_x = door_model[5]
  
--  local hand_rpy = Config.armfsm.dooropen.rhand_rpy
  local hand_rpy = {0,0,0,0,0,0}
  local hand_yaw = door_yaw
  if door_yaw>10*Body.DEG_TO_RAD then
    hand_yaw = math.max(0,20*Body.DEG_TO_RAD-door_yaw)
  end
--[[

print(unpack(hinge_pos))
print(door_yaw)
print(door_r)
print(knob_roll)
--]]

  local trHandle = T.eye()
    * T.trans(hinge_pos[1],hinge_pos[2],hinge_pos[3])    
    * T.rotZ(door_yaw)
    * T.trans(0,door_r, 0)         
    * T.rotZ(hand_yaw-door_yaw)
    * T.rotX(knob_roll)
    * T.transform6D(
      {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  

  local trTarget = T.position6D(trHandle)
  return trTarget
end

function movearm.getDoorEdgePosition(pos_offset,door_yaw)
  local door_model = hcm.get_door_model()  
  local hinge_pos = vector.slice(door_model,1,3) + vector.new(pos_offset)  
  hinge_pos[3] = hinge_pos[3] + Config.armfsm.dooredge.hinge_offset_z
  local door_r = door_model[4]

  local hand_offset_x = Config.armfsm.dooredge.hand_offset_x
  local edge_offset_x = Config.armfsm.dooredge.edge_offset_x
  local edge_offset_y = Config.armfsm.dooredge.edge_offset_y
 
  local hand_rpy = Config.armfsm.dooredge.rhand_rpy
  local hand_yaw = - (door_yaw - 25*Body.DEG_TO_RAD)

  if door_yaw>25*Body.DEG_TO_RAD then
    hand_yaw = math.min(hand_yaw * 2, 90*Body.DEG_TO_RAD)
  end

  local trHandle = T.eye()
    * T.trans(hinge_pos[1],hinge_pos[2],hinge_pos[3])    
    * T.rotZ(door_yaw)
    * T.trans(edge_offset_x, door_r+edge_offset_y, 0)             
    * T.rotZ(hand_yaw-door_yaw)    
    * T.transform6D(
      {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
    * T.trans(hand_offset_x,0,0)

  local trTarget = T.position6D(trHandle)
  return trTarget
end


--Use two chopstick hand
function movearm.getLargeValvePosition(turn_angleL,turn_angleR,offsetL,offsetR)
  local wheel   = hcm.get_wheel_model()
  local handle_pos    = vector.slice(wheel,1,3)
  local handle_yaw    = wheel[4]
  local handle_pitch  = 0 --we assume zero pitch 
  local handle_radius = wheel[6] 

  local lhand_rpy = {0,0*Body.DEG_TO_RAD, -10*Body.DEG_TO_RAD}
  local rhand_rpy = {0,0*Body.DEG_TO_RAD, 10*Body.DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  
  local trGripL = trHandle
       * T.rotX(turn_angleL)
       * T.trans(0,handle_radius,0)
       * T.transform6D(
          {0,0,0,lhand_rpy[1],lhand_rpy[2],lhand_rpy[3]})  
       * T.trans(offsetL,0,0)

  local trGripR = trHandle
       * T.rotX(turn_angleR)
       * T.trans(0,-handle_radius,0)
       * T.transform6D(
          {0,0,0,rhand_rpy[1],rhand_rpy[2],rhand_rpy[3]})  
       * T.trans(offsetR,0,0)       
  return T.position6D(trGripL), T.position6D(trGripR)
end

--Use LEFT chopstick hand
function movearm.getLargeValvePositionSingle(turn_angle,offset,is_left)
  local wheel = hcm.get_largevalve_model()
  local handle_pos = vector.slice(wheel,1,3)

  --we assume zero yaw and pitch 
  local handle_yaw    = 0
  local handle_pitch  = 0 
  local handle_radius = wheel[4] 
  local hand_rpy = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  if is_left>0 then
    local trGripL = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,handle_radius,0)
       * T.transform6D(
          {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
       * T.trans(offset,0,0)
    return T.position6D(trGripL)
  else
    local trGripR = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,-handle_radius,0)
       * T.transform6D(
          {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
       * T.trans(offset,0,0) 
     return T.position6D(trGripR)      
  end
end


--Use LEFT chopstick hand
function movearm.getBarValvePositionSingle(turn_angle,wrist_angle,offset)
  local wheel = hcm.get_barvalve_model()
  local handle_pos = vector.slice(wheel,1,3)
  local handle_radius = wheel[4]
  local handle_yaw,handle_pitch  = 0,0 --we assume zero yaw and pitch 
 
  --We assume verticle, downward valve as zero turnangle (and vertial chopsticks)
  local hand_rpy = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  
  local trGripL = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,0,-handle_radius)
       * T.rotX(wrist_angle)
       * T.transform6D(
          {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
       * T.trans(offset,0,0)
  return T.position6D(trGripL)
end


return movearm