-- Test SM for walk kick
-- Not for distribute


module(..., package.seeall);

require('Body')
require('vector')
require('Motion');
require('kick');
require('HeadFSM')
require('Config')
require('wcm')

require('walk');

t0 = 0;
timeout = 2.0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

  -- set kick depending on ball position
  ball = wcm.get_ball();
  if (ball.y > 0) then
    walk.doWalkKickLeft();
  else
    walk.doWalkKickRight();
  end
  HeadFSM.sm:set_state('headTrack');

--  HeadFSM.sm:set_state('headIdle');
end

function update()
  local t = Body.get_time();

  if (t - t0 > timeout) then
    return "done";
  end
end

function exit()
  HeadFSM.sm:set_state('headTrack');
end
