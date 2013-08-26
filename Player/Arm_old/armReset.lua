module(..., package.seeall);

require('lcm')
require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);

qLArmInit=Config.arm.qLArmInit;
qRArmInit=Config.arm.qRArmInit;


function entry()
  print(_NAME..' Entry' ) 

  left_arm_mode = 5;
  right_arm_mode = 5;
  Body.set_lhand_position(Config.arm.FingerOpen);
  Body.set_rhand_position(Config.arm.FingerOpen);

 --super slow
  dArmVelAngle = vector.new({10,10,10,15,45,45})*math.pi/180; --30 degree per second
  Body.set_arm_movement_velocity(dArmVelAngle);


end


function update()
--  print(_NAME..' Update' ) 

  if left_arm_mode==5 then
    Body.enable_larm_linear_movement(false); 
    Body.set_larm_target_position(qLArmInit[3]);
    if Body.larm_joint_movement_done() then left_arm_mode = 6;end
  elseif left_arm_mode==6 then
    Body.set_larm_target_position(qLArmInit[2]);
    if Body.larm_joint_movement_done() then left_arm_mode = 7;end
  elseif left_arm_mode==7 then
    Body.set_larm_target_position(qLArmInit[1]);
    if Body.larm_joint_movement_done() then left_arm_mode = 0;end
  end

  if right_arm_mode==5 then
    Body.enable_rarm_linear_movement(false); 
    Body.set_rarm_target_position(qRArmInit[3]);
    if Body.rarm_joint_movement_done() then right_arm_mode = 6;end
  elseif right_arm_mode==6 then
    Body.set_rarm_target_position(qRArmInit[2]);
    if Body.rarm_joint_movement_done() then right_arm_mode = 7;end
  elseif right_arm_mode==7 then
    Body.set_rarm_target_position(qRArmInit[1]);
    if Body.rarm_joint_movement_done() then right_arm_mode = 0;end
  end

  if left_arm_mode==0 and right_arm_mode==0 then
    return "done";
  end
end

function exit()
  print(_NAME..' Exit' ) 

end
