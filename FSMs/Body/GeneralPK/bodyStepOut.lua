module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')
local vector = require('vector')
local util = require('util')
local Config = require('Config')
local wcm = require('wcm')
local gcm = require('gcm')

t0 = 0;

tStepOut = 5.0;
function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
end

function update()
  local t = Body.get_time();
  walk.set_velocity(0.04,0,0);

  if (t-t0>tStepOut) then
    return "done";
  end
end

function exit()
  walk.stop();
end

