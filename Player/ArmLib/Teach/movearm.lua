local movearm={}
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
local P = require'libPlan'
require'hcm'

local lShoulderYaw = -45*DEG_TO_RAD;
local rShoulderYaw = 45*DEG_TO_RAD;

local lPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]),
	vector.slice(Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]))
local rPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]), 
	vector.slice(Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]))

local dqLimit = DEG_TO_RAD / 3

	-- Body.get_inverse_lwrist:
	--[[
  return Kinematics.inverse_l_wrist(trL, qL, lShoulderYaw or qL[3],
      bodyTilt or mcm.get_stance_bodyTilt(), qWaist or Body.get_waist_command_position())
	--]]

function movearm.goto_wrists(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
	  local qLArm = Body.get_larm_position()
	  local qLWrist = Body.get_inverse_lwrist(qLArm, unpack(lwrist, 1, 2))
	  local qLGoal = Body.get_inverse_arm_given_wrist(qLWrist, lwrist[3])
		lPathIter = lPlanner:joint_iter(qLGoal, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_position()
	  local qRWrist = Body.get_inverse_rwrist(qRArm, unpack(rwrist, 1, 2))
	  local qRGoal = Body.get_inverse_arm_given_wrist(qRWrist, rwrist[3])
		rPathIter = rPlanner:joint_iter(qRGoal, dqLimit)
	end
	return lPathIter, rPathIter
end

-- Take in 6D Transform
function movearm.goto_tr6(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
		local qLArm = Body.get_larm_command_position()
		local qWaist = Body.get_waist_command_position()
		local q_lGoal = Kinematics.inverse_l_arm_7(lwrist,qRArm,0,0,qWaist)
		lPathIter = lPlanner:joint_iter(q_lGoal, dqLimit)
	end
	if rwrist then
		local qRArm = Body.get_rarm_command_position()
		local qWaist = Body.get_waist_command_position()
		local q_rGoal = Kinematics.inverse_r_arm_7(rwrist,qRArm,0,0,qWaist)
		rPathIter = rPlanner:joint_iter(q_rGoal, dqLimit)
	end
	return lPathIter, rPathIter
end

-- Take in Angles
function movearm.goto_q(lwrist, rwrist)
	local lPathIter, rPathIter
	if lwrist then
		lPathIter = lPlanner:joint_iter(lwrist, dqLimit)
	end
	if rwrist then
		rPathIter = rPlanner:joint_iter(rwrist, dqLimit)
	end
	return lPathIter, rPathIter
end

function movearm.setArmJoints(qLArmTarget,qRArmTarget, dt,dqArmLim)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()  

  local dqVelLeft = mcm.get_arm_dqVelLeft()
  local dqVelRight = mcm.get_arm_dqVelRight()

  local qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqVelLeft, dt )  
  local qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqVelRight, dt )

  -- TODO: Dynamixel is STUPID so we should manually check for the direction
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

function movearm.getDoorHandlePosition(pos_offset,knob_roll,door_yaw, is_left)
  local door_model = hcm.get_door_model()  
  local hinge_pos = vector.slice(door_model,1,3) + vector.new(pos_offset)  
  local door_r = door_model[4]
  local grip_offset_x = door_model[5]
  local knob_offset_y = door_model[6]

  local hand_yaw = door_yaw
  local yaw_factor = 2.5
  local hand_rpy
--  local yaw_factor = 2 

  if is_left and is_left>0 then
    hand_rpy = Config.armfsm.dooropen.lhand_rpy
    hand_yaw = 0
  else
    hand_rpy = Config.armfsm.dooropen.rhand_rpy
    if door_yaw<0 then
      hand_yaw = 0
    elseif door_yaw>10*DEG_TO_RAD then
      hand_yaw = door_yaw-(door_yaw-10*DEG_TO_RAD)*yaw_factor
    end
  end
 
  local trHandle = T.eye()
    * T.trans(hinge_pos[1],hinge_pos[2],hinge_pos[3])    
    * T.rotZ(door_yaw)
    * T.trans(grip_offset_x, door_r, 0)     
    * T.rotX(knob_roll)
    * T.trans(0,knob_offset_y, 0) 
    * T.rotX(-knob_roll)
    * T.rotZ(hand_yaw-door_yaw)
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
  local hand_yaw = - (door_yaw - 25*DEG_TO_RAD)

  if door_yaw>25*DEG_TO_RAD then
    hand_yaw = math.min(hand_yaw * 2, 90*DEG_TO_RAD)
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

  local lhand_rpy = {0,0*DEG_TO_RAD, -10*DEG_TO_RAD}
  local rhand_rpy = {0,0*DEG_TO_RAD, 10*DEG_TO_RAD}

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
  local hand_rpy = {0,0*DEG_TO_RAD, 0*DEG_TO_RAD}

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
  local hand_rpy = {0,0*DEG_TO_RAD, 0*DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  
  local trGripL = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,0,handle_radius)
       * T.rotX(wrist_angle)
       * T.transform6D(
          {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
       * T.trans(offset,0,0)
  return T.position6D(trGripL)
end

function movearm.getHoseAttachPosition(offset, wristRollL, wristRollR, rhand_clearance)
  local model = hcm.get_hoseattach_model()
  local lhand_rpy = Config.armfsm.hoseattach.lhand_rpy
  local rhand_rpy = Config.armfsm.hoseattach.rhand_rpy
  local lhand_offset = Config.armfsm.hoseattach.lhand_offset
  local rhand_offset = Config.armfsm.hoseattach.rhand_offset

  local trHose = T.eye()
       * T.trans(model[1]+offset[1],model[2]+offset[2],model[3]+offset[3])
       * T.rotZ(model[4])
       * T.rotY(model[5])

  local trLeft = trHose
      *T.trans(lhand_offset[1],lhand_offset[2],lhand_offset[3])
      *T.rotX(wristRollL)
      * T.transform6D(
          {0,0,0,lhand_rpy[1],lhand_rpy[2],lhand_rpy[3]})  

  local trRight = trHose
      *T.trans(rhand_offset[1],rhand_offset[2],rhand_offset[3])
      *T.rotX(wristRollR)
      *T.trans(0,-rhand_clearance,0)
      *T.rotX(-wristRollR)
      * T.transform6D(
          {0,0,0,rhand_rpy[1],rhand_rpy[2],rhand_rpy[3]})  

  return T.position6D(trLeft),T.position6D(trRight)
end

return movearm
