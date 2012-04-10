module(..., package.seeall);

require('carray');
require('vector');
require('rlcm')

function set_next_policy()
  -- Get our current policy
  -- Get our current stage of evaluation
  -- (1,2,3,...nParams)
  local stage = rlcm.get_trial_stage();
  print('params',#rlcm.enum_param,stage)
  if( stage<=#rlcm.enum_param ) then
    -- If in a gradient sampling stage, then just pick the next sample    
    -- Sample in this stage direction...
    local param2tune = rlcm.enum_param[stage];
    print('Gradient Sampling Stage',stage,param2tune)
    local paramVal = rlcm['get_params_'..param2tune]();
    local paramDelta = 0.01; -- TODO: Should be different for each parameter!
    rlcm['set_params_'..param2tune]( paramVal+paramDelta );

    -- Reset previous parameter
    if( stage ~=1 ) then
      local old_param2tune = rlcm.enum_param[stage-1];
      local old_paramVal = rlcm['get_params_'..old_param2tune]();
      local old_paramDelta = 0.01; -- TODO: Should be different for each parameter!
      rlcm['set_params_'..old_param2tune]( old_paramVal-old_paramDelta );
    end

    rlcm.set_trial_stage( stage+1 );
  else
    print('Exploit a gradient')
    -- If in gradient exploitation step, then pick policy + gradient * gradient_step_size
    gradient = 0; -- TODO: calculate gradient...
    rlcm.set_trial_stage( 1 ); -- Sample in this direction next
  end

end
