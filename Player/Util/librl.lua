module(..., package.seeall);

require('carray');
require('vector');
require('rlcm')

function set_next_policy()
  -- Get our current policy
  -- Get our current stage of evaluation
  -- (1,2,3,...nParams)
  local stage = rlcm.get_trial_stage();
  if( stage<=#rlcm.enum_param ) then
    -- If in a gradient sampling stage, then just pick the next sample    
    -- Sample in this stage direction...
    local param2tune = rlcm.enum_param[stage];
    print('Gradient Sampling Stage',stage,param2tune)
    local paramVal = rlcm['get_params_'..param2tune]();
    local paramDelta = 0.01; -- TODO: Should be different for each parameter!
    rlcm['set_params_'..param2tune]( paramVal+paramDelta );

    -- Reset previous parameter
    if( stage ~= 1 ) then
      local old_param2tune = rlcm.enum_param[stage-1];
      local old_paramVal = rlcm['get_params_'..old_param2tune]();
      local old_paramDelta = 0.01; -- TODO: Should be different for each parameter!
      rlcm['set_params_'..old_param2tune]( old_paramVal-old_paramDelta );
    end
  else
    print('Exploit a gradient')
    -- If in gradient exploitation step, then pick policy + gradient * gradient_step_size
    rewards = rlcm.get_trial_reward();
    gradient = vector.zeros(#rlcm.enum_param);
    for p=1,#rlcm.enum_param do
      gradient[p] = rewards[p];
    end
    gradient = gradient / vector.norm(gradient);
    for p=1,#rlcm.enum_param do
      local paramDelta = 0.01; -- TODO: Should be different for each parameter!
      gradient[p] = rewards[p] * paramDelta;
    end
    print('Gradient',gradient)
    for p=1,#rlcm.enum_param do
      local param2tune = rlcm.enum_param[p];
      local paramVal = rlcm['get_params_'..param2tune]();
      local gradStep = 0.1; -- TODO tune
      rlcm['set_params_'..param2tune]( paramVal+gradient[p]*gradStep );
    end
  end

end

function record_reward(gps0,gps1)

  gps_diff = {gps1[1]-gps0[1],gps1[2]-gps0[2],gps1[3]-gps0[3] };
  dist_traveled = vector.norm(vector.new(gps_diff));
  -- Get current rewards
  rewards = rlcm.get_trial_reward();
  stage = rlcm.get_trial_stage();
  rewards[stage] = dist_traveled;
  rlcm.set_trial_reward( rewards );  
  set_next_stage();
end

function record_fall()
  -- Get current rewards
  rewards = rlcm.get_trial_reward();
  stage = rlcm.get_trial_stage();
  rewards[stage] = 0; -- all dist are > 0.  Going nowhere is a big penalty
  rlcm.set_trial_reward( rewards );
  set_next_stage();
end

function set_next_stage()
  local stage = rlcm.get_trial_stage();
  if( stage<=#rlcm.enum_param ) then
    stage = stage + 1;
  else
    stage = 1; -- Sample in this direction next
  end
  rlcm.set_trial_stage( stage );  
  -- Zero reward until completed...
  rewards = rlcm.get_trial_reward();
  rewards[stage] = 0;
  rlcm.set_trial_reward( rewards );  
end
