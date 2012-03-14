module(..., package.seeall);

require('Body')
require('wcm')
require('walk')

t0 = 0;
timeout = 10.0;

-- turn velocity
vSpin = 0.3;
direction = 1;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();

  -- set turn direction to last known ball position
  stretcher = wcm.get_stretcher();
  if (stretcher.y > 0) then
    direction = 1;
  else
    direction = -1;
  end
end

function update()
  local t = Body.get_time();

  stretcher = wcm.get_stretcher();

  -- search/spin until the ball is found
  walk.set_velocity(0, 0, direction*vSpin);

  if (t - stretcher.t < 0.1) then
    return "stretcher";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
end
