module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')
local vector = require('vector')

local wcm = require('wcm')
local gcm = require('gcm')

t0 = 0;
timeout = 3.0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  walk.set_velocity(0,0,0);
  walk.stop();
  Speak.talk('Defending');
end

function update()
  local t = Body.get_time();
  walk.stop();

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
  walk.start();
end

