module(..., package.seeall);

require('Body')
require('vector')
require('walk');
require('wcm');

t0 = 0;
timeout = 20.0;
started = false;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

end

function update()
  local t = Body.get_time();

  -- Check if the other robot is in position...
  if( wcm.get_opponent_ready() ) then
    return 'done';
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
   walk.start();
end
