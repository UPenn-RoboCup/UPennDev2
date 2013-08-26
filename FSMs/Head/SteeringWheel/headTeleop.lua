module(..., package.seeall);
Config = require('ConfigPenn')
Body = require(Config.Body)
require('hcm')
RAD = math.pi/180;

function entry()
  print(_NAME..' Entry' ) 

  lcm:set_head_lidar_panning(0);
  t0 = unix.time();
end

function update()
--  print(_NAME..' Update' ) 

  t = unix.time();
  local tdiff = t-t0;
  local neck0 = hcm:get_control_head_movement();
  local neck = {neck0[1]*60*RAD, neck0[2]*60*RAD};
  Body.set_neck_target_position(neck);
  t0=t;
end

function exit()
  lcm:set_head_lidar_panning(0);
  print(_NAME..' Exit' ) 
end
