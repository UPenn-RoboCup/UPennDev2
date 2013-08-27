module(..., package.seeall);

require('hcm')
require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);

local Kinematics = require'Kinematics'

armpos_resolution = 0.001;


function entry()
  qLArm = Body.get_larm_command_position();
  qRArm = Body.get_rarm_command_position();
  trLArmTarget = Kinematics.l_arm_torso(qLArm); 
  trRArmTarget = Kinematics.r_arm_torso(qRArm); 
  trLArmTarget0=Kinematics.l_arm_torso(qLArm)
  trRArmTarget0=Kinematics.r_arm_torso(qRArm)
  larm_mode_command_old = {0};
  rarm_mode_command_old = {0};
  larm_movement_old={0,0,0};
  rarm_movement_old={0,0,0};
  print(_NAME..' Entry' ) 
end


function update()
--  print(_NAME..' Update' ) 

  local larm_mode_command = hcm:get_control_left_arm_mode();
  local rarm_mode_command = hcm:get_control_right_arm_mode();

  if larm_mode_command[1]==9 then --2nd trigger pulled, enable arm movement
    local lgripper_command = hcm:get_control_left_gripper();
    Body.set_lhand_position(lgripper_command[1]);
    Body.enable_larm_linear_movement(true); 

    if larm_mode_command_old[1]==0 then --Button just pressed
      --Store current controller position
      larm_movement_old = hcm:get_control_left_arm_movement();
      --Store initial arm transform
      trLArmTarget0={trLArmTarget[1],trLArmTarget[2],trLArmTarget[3],
        trLArmTarget[4],trLArmTarget[5],trLArmTarget[6]}
    end
    --Now get curret reading
    local larm_movement = hcm:get_control_left_arm_movement();
    local trTemp ={
		  trLArmTarget0[1]+  (larm_movement[1]-larm_movement_old[1])*armpos_resolution,
		  trLArmTarget0[2]+  (larm_movement[2]-larm_movement_old[2])*armpos_resolution,
		  trLArmTarget0[3]+  (larm_movement[3]-larm_movement_old[3])*armpos_resolution,
		  trLArmTarget0[4],
		  trLArmTarget0[5],
		  trLArmTarget0[6]}
    local done = Body.set_larm_target_transform(trTemp);
    if done then trLArmTarget = trTemp;   --IK possible
    else   Body.set_larm_target_transform(trLArmTarget); --IK not possible, don't move
    end
  else --2nd trigger released, stop every movement
    Body.enable_larm_linear_movement(false); 
    qLArm = Body.get_larm_command_position();
    trTemp = Kinematics.l_arm_torso(qLArm); 
    trLArmTarget[1],trLArmTarget[2],trLArmTarget[3]= trTemp[1],trTemp[2],trTemp[3];
    Body.set_larm_target_transform(trLArmTarget);
  end


  if rarm_mode_command[1]==9 then --2nd trigger pulled, enable arm movement
    local rgripper_command = hcm:get_control_right_gripper();
    Body.set_rhand_position(rgripper_command[1]);

    if rarm_mode_command_old[1]==0 then --Button just pressed
      Body.enable_rarm_linear_movement(true); 
      --Store current controller position
      rarm_movement_old = hcm:get_control_right_arm_movement();
      --Store current arm transform
      trRArmTarget0={trRArmTarget[1],trRArmTarget[2],trRArmTarget[3],
        trRArmTarget[4],trRArmTarget[5],trRArmTarget[6]}
      end
      --Now get curret reading
      local rarm_movement = hcm:get_control_right_arm_movement();
      local trTemp ={
			  trRArmTarget0[1]+ (rarm_movement[1]-rarm_movement_old[1])*armpos_resolution,
			  trRArmTarget0[2]+ (rarm_movement[2]-rarm_movement_old[2])*armpos_resolution,
			  trRArmTarget0[3]+ (rarm_movement[3]-rarm_movement_old[3])*armpos_resolution,
			  trRArmTarget0[4],
			  trRArmTarget0[5],
			  trRArmTarget0[6]}

      local done = Body.set_rarm_target_transform(trTemp);
      if done then  trRArmTarget = trTemp; --IK possible
      else  Body.set_rarm_target_transform(trRArmTarget); --Ik not possible
      end

  else

     Body.enable_rarm_linear_movement(false); 
     qRArm = Body.get_rarm_position();
     trTemp = Kinematics.r_arm_torso(qRArm); 
     trRArmTarget[1],trRArmTarget[2],trRArmTarget[3]=
		  trTemp[1],trTemp[2],trTemp[3];
     Body.set_rarm_target_transform(trRArmTarget);
  end

  larm_mode_command_old[1] = larm_mode_command[1];
  rarm_mode_command_old[1] = rarm_mode_command[1];
end

function exit()

  print(_NAME..' Exit' ) 

end
