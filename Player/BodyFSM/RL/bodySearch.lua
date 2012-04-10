module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require 'vector'

require 'mcm'
require 'rlcm' -- Contains current state info and learning rates, etc.
require 'librl'

t0 = 0;
timeout = 10.0; -- Ten second trial!

gps_init = {}
gps_final = {};

function entry()
  print("Body FSM:".._NAME.." entry");

  local trialnum = rlcm.get_trial_num();
  print('\n=====')
  print('Trial',trialnum);
  Motion.event("walk");

  -- Record starting GPS
  gps_init = Body.get_sensor_gps();

  -- Load new walking parameters
  for param,value in pairs(rlcm.shared.params) do
    rl_val = rlcm['get_params_'..param]();
    print('Loading',param,rl_val)
    --  mcm['set_walk_'..param]( rl_val );
    --  MCM is not used...
    if( param~='vx' ) then
      walk[param] = rl_val; -- Update value in walking engine
    end
  end

  t0 = Body.get_time();
  walk.start();
  local vx = rlcm.get_params_vx();
  walk.set_velocity( vx, 0, 0 );
end

function update()
  local t = Body.get_time();

  local falling = mcm.get_walk_isFallDown();
  if( falling==1 ) then
    return 'fall';
  end

  if (t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
  print('Done trial!  Recording results...')
  -- Record starting GPS
  gps_final = Body.get_sensor_gps();
  local falling = mcm.get_walk_isFallDown();
  if( falling==0 ) then
    librl.record_reward( gps_init, gps_final );
  else
    librl.record_fall( );
  end
end

