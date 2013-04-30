module(..., package.seeall);

local Body = require('Body')
local Motion = require('Motion')

t0 = 0;

function entry()
  print(_NAME..' entry');
  t0 = Body.get_time();
  
  Motion.event("sit");
--  Motion.sm:set_state('resit');
--  Motion.sm:set_state('standstill');
end

function update()
  Motion.event("sit");
  t = Body.get_time();
end

function exit()
  Motion.sm:set_state('stance');
end
