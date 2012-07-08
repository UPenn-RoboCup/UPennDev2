module(..., package.seeall);

require('Body')
require('walk')
require('vector')

require('wcm')
require('gcm')

t0 = 0;
timeout = 6.0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  walk.set_velocity(0,0,0);
  walk.stop();
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

