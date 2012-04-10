module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require 'vector'

require 'mcm'
require 'rlcm' -- Contains current state info and learning rates, etc.

t0 = 0;
timeout = 5.0; -- Ten second trial!

function entry()
  print("Body FSM:".._NAME.." entry");

  -- Load new walking parameters
  for param,value in pairs(rlcm.shared.params) do
    rl_val = rlcm['get_params_'..param]();
    print('Loading',param,rl_val)
    --  mcm['set_walk_'..param]( rl_val );
    --  MCM is not used...
    walk[param] = rl_val; -- Update value in walking engine
  end

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
  print('Done trial!')
  walk.stop();
end

