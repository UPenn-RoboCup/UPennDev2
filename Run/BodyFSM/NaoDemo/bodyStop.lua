module(..., package.seeall);

local Body = require('Body')
local Motion = require('Motion')

function entry()
  print(_NAME..' entry');

  walk.set_velocity(0,0,0);
  walk.stop();
end

function update()
  -- do nothing
end

function exit()
  walk.start();
end
