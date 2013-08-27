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
end

function update()
--  print(_NAME..' Update' ) 
    Body.enable_larm_linear_movement(false); 
--    Body.set_larm_target_position(qLArmInit[#qLArmInit]);
end

function exit()
  print(_NAME..' Exit' ) 

end
