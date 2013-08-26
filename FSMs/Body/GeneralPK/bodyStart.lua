module(..., package.seeall);

local walk = require('walk')
local gcm = require('gcm')
local Config = require('Config')

function entry()
  print(_NAME..' entry');
end

function update()
  return 'done';
end


function exit()
  walk.stop()
end
