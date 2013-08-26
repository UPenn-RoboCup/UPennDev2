-- Test SM for walk kick
-- Not for distribute


module(..., package.seeall);

local Body = require('Body')
local vector = require('vector')
local Motion = require('Motion');
local kick = require('kick');
local HeadFSM = require('HeadFSM')
local Config = require('Config')
local wcm = require('wcm')

local walk = require('walk');

t0 = 0;
timeout = Config.fsm.bodyWalkKick.timeout;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  follow=false;
  kick_dir=wcm.get_kick_dir();


print("KICK DIR:",kick_dir)

  if kick_dir==1 then --straight walkkick
    -- set kick depending on ball position
    ball = wcm.get_ball();
    if (ball.y > 0) then
      walk.doWalkKickLeft();
    else
      walk.doWalkKickRight();
    end
  elseif kick_dir==2 then --sidekick to left
    walk.doSideKickLeft();
  else
    walk.doSideKickRight(); --sidekick to right
  end
  HeadFSM.sm:set_state('headTrack');
--  HeadFSM.sm:set_state('headIdle');
end

function update()
  local t = Body.get_time();

  if (t - t0 > timeout) then
    return "done";
  end

  --SJ: should be done in better way?
  if walk.walkKickRequest==0 and follow ==false then
    follow=true;
    HeadFSM.sm:set_state('headKickFollow');
  end

end

function exit()
 -- HeadFSM.sm:set_state('headTrack');
end
