module(..., package.seeall);

require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);

qLArmInit=Config.arm.qLArmInit;
qRArmInit=Config.arm.qRArmInit;

function entry()
  print(_NAME..' Entry' ) 
  Body.enable_larm_linear_movement(false); 
  Body.set_larm_target_position(qLArmInit[1]);
  Body.set_rarm_target_position(qRArmInit[1]);
  Body.set_lhand_position(Config.arm.FingerOpen);
  Body.set_rhand_position(Config.arm.FingerOpen);
end

function update()
--  print(_NAME..' Update' ) 

end

function exit()
  print(_NAME..' Exit' ) 

end
