module(..., package.seeall);

require('Body')
require('vector')
require('walk');
require('wcm');

t0 = 0;
timeout = 15.0;
started = false;

function entry()
  print(_NAME.." entry");
  wcm.set_agent_ready(1);

  t0 = Body.get_time();

end

function update()
  local t = Body.get_time();

  -- Check if the other robot is in position...
  local opp_ready = wcm.get_opponent_ready();
  if( opp_ready==1 ) then
    return 'done';
  end
  
  if (t - t0 > timeout) then
--    print('opponent not ready...')
    return "timeout";
  end
end

function exit()
   walk.start();
end
