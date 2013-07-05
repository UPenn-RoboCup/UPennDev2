module(..., package.seeall);

require('lcm')
require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);

qLArmInit=Config.arm.qLArmInit;
qRArmInit=Config.arm.qRArmInit;


function entry()
  print(_NAME..' Entry' ) 
  left_arm_mode = 1;
  right_arm_mode = 1;
  
   --super slow
  dArmVelAngle = vector.new({10,10,10,15,45,45})*math.pi/180; --30 degree per second
  Body.set_arm_movement_velocity(dArmVelAngle);


end


function update()
--  print(_NAME..' Update' ) 


  if left_arm_mode==1 then
    Body.enable_larm_linear_movement(false); 
    Body.set_larm_target_position(qLArmInit[1]);
    if Body.larm_joint_movement_done() then left_arm_mode = 2;end
  elseif left_arm_mode==2 then
    Body.set_larm_target_position(qLArmInit[2]);
    if Body.larm_joint_movement_done() then left_arm_mode = 3;end
  elseif left_arm_mode==3 then
    Body.set_larm_target_position(qLArmInit[3]);
    if Body.larm_joint_movement_done() then 
      left_arm_mode = 4;
    end
  end

  if left_arm_mode==4 then
    return "done";
  end
end

function exit()
  print(_NAME..' Exit' ) 

end
