module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require 'librl'

t0 = 0;
timeout = 2.0;
fell = 0;

t1 = nil;

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  print('Prep time! (Basically just stop)');
  walk.stop();

  -- Select the next policy
  librl.set_next_policy();

  fell = mcm.get_walk_isFallDown();
end

function update()
  local t = Body.get_time();

  falling = mcm.get_walk_isFallDown();
  if( fell==1 and falling==0 ) then
    -- Wait for it to get up...
    if( not t1 ) then
      Motion.event('standstill');
      t1 = Body.get_time();
      timeout = 6.0;
    elseif(t-t1>timeout)then
      return 'timeout';
    end
  end

  if ( fell==0 and falling==0 and t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
end

