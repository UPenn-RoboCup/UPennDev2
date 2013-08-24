module(..., package.seeall);

require('unix')
require('hcm')
Body = require(Config.Body);

function entry()
  print(_NAME..' Entry' ) 
  Body.set_lwheel_velocity(0);
  Body.set_rwheel_velocity(0);
end

function update()
--  print(_NAME..' Update' ) 
  local wheelVel = hcm:get_control_wheel_velocity();
  Body.set_lwheel_velocity(wheelVel[1]);
  Body.set_rwheel_velocity(wheelVel[2]);
end

function exit()
  print(_NAME..' Exit' ) 
  Body.set_lwheel_velocity(0);
  Body.set_rwheel_velocity(0);
end

