module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require 'librl'

t0 = 0;
timeout = 2.0;

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  print('Prep time! (Basically just stop)')
  walk.stop();

  -- Select the next policy
  librl.set_next_policy();
end

function update()
  local t = Body.get_time();

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
end

