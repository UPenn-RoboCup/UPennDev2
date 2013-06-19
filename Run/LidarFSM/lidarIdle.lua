module(..., package.seeall);

require('lcm')
require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);

function entry()
  lcm:set_chest_lidar_panning(0);
  print(_NAME..' Entry' ) 
end

function update()
--  print(_NAME..' Update' ) 
end

function exit()
  print(_NAME..' Exit' ) 
end
