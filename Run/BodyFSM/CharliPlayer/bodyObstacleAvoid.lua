module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')
local vector = require('vector')
local UltraSound = require('UltraSound')

local wcm = require('wcm')
local gcm = require('gcm')

t0 = 0;
timeout = 5.0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

  Speak.talk('Avoiding Obstacle');
  us = UltraSound.check_obstacle();
  direction = 1;
  if us[1] > us[2] then
    direction = -1;
  end
end

function update()
  local t = Body.get_time();
  walk.set_velocity(0, direction*0.04, 0);

  us = UltraSound.check_obstacle();
  if (us[1] < 7 and us[2] < 7) then
    return 'clear';
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
  walk.start();
end

