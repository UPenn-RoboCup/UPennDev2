module(..., package.seeall);

local Body = require('Body')
local Motion = require('Motion')

t0 = 0;

function entry()
  print(_NAME..' entry');

  t0 = Body.get_time();

  Motion.sm:set_state('sit');
end

function update()
  t = Body.get_time();
end

function exit()
  Motion.sm:set_state('stance');
end
