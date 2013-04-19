module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')

function entry()
  print(_NAME..' entry');
  
  walk.start();
end

function update()
  return 'done';
end

function exit()
end
