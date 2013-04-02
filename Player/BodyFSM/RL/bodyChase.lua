module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')
require 'Config'
require 'rlcm'

t0 = 0;
timeout = 20.0;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();

  -- Reset RL parameters for each new run
  for param,value in pairs(rlcm.shared.params) do
    if( param~='vx' ) then
      local config_val = Config.optimal[param];
      rlcm['set_params_'..param](config_val);
      print('Optimal setting',param,config_val);
      walk[param] = config_val;
    else
      rlcm.set_params_vx(Config.optimal['vx']);
      print('Resetting\tvx\t0.1');
    end
  end
  rlcm.set_trial_num(0);
  rlcm.set_trial_stage(#rlcm.enum_param+1);
  walk.start();
end

function update()
  local t = Body.get_time();

  walk.set_velocity(rlcm.get_params_vx(),0,0);

  if (t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
end

