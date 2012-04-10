module(..., package.seeall);

require('Body')
require('World')
require('vector')
require('Motion');
require('grip');
require('walk');
require('wcm');

t0 = 0;
timeout = 20.0;
started = false;

--by SJ
kickable=true;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

  -- set kick depending on ball position

  local ball = wcm.get_ball();
  if( t0 - ball.t < .5 ) then -- Just saw it
	  grip.setdistance( ball.x );
  end

  --SJ - only initiate kick while walking
  kickable = walk.active;  
  grip.throw = 0;
  Motion.event("grip");
  started = false;
end

function update()
  local t = Body.get_time();
  if not kickable then 
   print("bodygrip escape");
   return "done";
  end
  
  if (not started and grip.active) then
    started = true;
  elseif (started and not grip.active) then
    return "done";
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
   walk.start();
end
