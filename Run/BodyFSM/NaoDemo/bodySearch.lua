module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')
local vector = require('vector')
local Config = require('Config')
local wcm = require('wcm')

t0 = 0;
timeout = 10.0;

-- turn velocity
vSpin = 0.3;

direction = 1;


function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

  -- set turn direction to last known ball position
  ball = wcm.get_ball();
  if (ball.y > 0) then
    direction = 1;
  else
    direction = -1;
  end
end

function update()
  local t = Body.get_time();

  ball = wcm.get_ball();

  -- search/spin until the ball is found
  walk.set_velocity(0, 0, direction*vSpin);

  if (t - ball.t < 0.1) then
    return "ball";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
end
