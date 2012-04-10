module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require 'vector'

require 'mcm'
require 'rlcm' -- Contains current state info and learning rates, etc.

t0 = 0;
timeout = 10.0;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();
  walk.start();
end

function update()
  local t = Body.get_time();

  walk.set_velocity(0.04, 0, 0);

  if (t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
  walk.stop();  
end

