module(..., package.seeall);

require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);

qLArmInit=Config.arm.qLArmInit;
qRArmInit=Config.arm.qRArmInit;

function entry()
  print(_NAME..' Entry' ) 
  Body.enable_larm_linear_movement(false); 
  Body.enable_rarm_linear_movement(false); 
  
   --super slow
  dArmVelAngle = vector.new({10,10,10,15,45,45})*math.pi/180; --30 degree per second
  Body.set_arm_movement_velocity(dArmVelAngle);

end

function update()
--  print(_NAME..' Update' ) 
    Body.enable_larm_linear_movement(false); 
    Body.set_larm_target_position(qLArmInit[#qLArmInit]);
    Body.set_rarm_target_position(qRArmInit[#qRArmInit]);
--  print(_NAME..' Update' ) 
end

function exit()
  print(_NAME..' Exit' ) 

end
