module(..., package.seeall);

require('carray');
require('vector');
require('rlcm')

-- For logging
saveCount = 0;

function set_next_policy()
  -- Get our current policy
  -- Get our current stage of evaluation
  -- (1,2,3,...nParams)
  local stage = rlcm.get_trial_stage();
  if( stage<=#rlcm.enum_param ) then
    -- If in a gradient sampling stage, then just pick the next sample    
    -- Sample in this stage direction...
    local param2tune = rlcm.enum_param[stage];
    print('Gradient Sampling Stage ',stage,param2tune)
    local paramVal = rlcm['get_params_'..param2tune]();
    local paramDelta = Config.rl['d_'..param2tune];
    rlcm['set_params_'..param2tune]( paramVal+paramDelta );

    -- Reset previous parameter
    if( stage ~= 1 ) then
      local old_param2tune = rlcm.enum_param[stage-1];
      local old_paramVal = rlcm['get_params_'..old_param2tune]();
      local old_paramDelta = Config.rl['d_'..old_param2tune];
      rlcm['set_params_'..old_param2tune]( old_paramVal-old_paramDelta );
    end
  else
    print('Exploit a gradient')
    -- If in gradient exploitation step, then pick policy + gradient * gradient_step_size
    rewards = rlcm.get_trial_reward();
    
    gradient = vector.zeros(#rlcm.enum_param);
    for p=1,#rlcm.enum_param do
      gradient[p] = rewards[p]-rewards[#rlcm.enum_param+1]; -- Less the original point
    end
    gradient = gradient / vector.norm(gradient);

    print('\nRewards',rewards)
    print('Gradient',gradient)
    for p=1,#rlcm.enum_param do
      local param2tune = rlcm.enum_param[p];
      local paramVal = rlcm['get_params_'..param2tune]();
      local gradStep = Config.rl['g_'..param2tune];
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
  print('\n\tReward:',dist_traveled,'\n');
end

function record_fall()
  for i=1,20 do
    print('recording a fall!');
  end
  -- Get current rewards
  rewards = rlcm.get_trial_reward();
  stage = rlcm.get_trial_stage();
  rewards[stage] = -1; -- all dist are > 0.  Going nowhere is a big penalty
  rlcm.set_trial_reward( rewards );
  set_next_stage();
end

function set_next_stage()
  local stage = rlcm.get_trial_stage();
  print('Current stage ',stage)
  if( stage<=#rlcm.enum_param ) then
    stage = stage + 1;
  else
    log_rl();
    local trialnum = rlcm.get_trial_num();
    -- Increase Trial Number
    rlcm.set_trial_num( trialnum+1 );
    stage = 1; -- Sample in this direction next
  end
  rlcm.set_trial_stage( stage );  
  -- Zero reward until completed...
  rewards = rlcm.get_trial_reward();
  rewards[stage] = 0;
  rlcm.set_trial_reward( rewards );  
end

function log_rl( )
  local logfile_name = string.format("/tmp/rl_data.raw");
  -- Open the file
  if( saveCount == 0 ) then  
    f = io.open(logfile_name, "w");
  else
    f = io.open(logfile_name, "a");  
  end
  assert(f, "Could not open save image file");

  if( saveCount == 0 ) then
    -- Write the Header
    f:write( "trialnum,reward" );
    for p=1,#rlcm.enum_param do
      local param2tune = rlcm.enum_param[p];
      f:write( string.format(",%s",param2tune) );
    end
    f:write( "\n" );
  end

  -- Write the data
  local trialnum = rlcm.get_trial_num();
  local rewards = rlcm.get_trial_reward();
  f:write( string.format("%d",trialnum) );
  f:write( string.format(",%f",rewards[#rlcm.enum_param+1]) );
  for p=1,#rlcm.enum_param do
    local param2tune = rlcm.enum_param[p];
    local paramVal = rlcm['get_params_'..param2tune]();
    f:write( string.format(",%f",paramVal) );
  end
  f:write( "\n" );
  -- Close the file
  f:close();
  saveCount = saveCount + 1;

end

