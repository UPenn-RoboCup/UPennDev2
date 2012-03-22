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
tStart = 0;
timeout = 20.0;

started = false;
kickable = true;
follow = false;

tFollowDelay = Config.fsm.bodyKick.tFollowDelay;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

  kick_dir=wcm.get_kick_dir();
  if kick_dir==1 then
    -- straight kick, set kick depending on ball position
    ball = wcm.get_ball();
    if (ball.y > 0) then
      kick.set_kick("kickForwardLeft");
    else
      kick.set_kick("kickForwardRight");
    end
  elseif kick_dir==2 then --Kick to left
      kick.set_kick("kickSideRight");
  else --Kick to right
      kick.set_kick("kickSideLeft");
  end

  --SJ - only initiate kick while walking
  kickable = walk.active;  

  Motion.event("kick");
  started = false;
  follow = false;

end

function update()
  local t = Body.get_time();
  if not kickable then 
     print("bodyKick escape");
     --Set velocity to 0 after kick fails ot prevent instability--
     walk.setVelocity(0, 0, 0);
     return "done";
  end
  
  if (not started and kick.active) then
    started = true;
    tStart =t;
  elseif started then
    if kick.active then
      if follow==false and t-tStart > tFollowDelay then
  	HeadFSM.sm:set_state('headKickFollow');
	follow=true;
      end
    else --Kick ended
  	--Set velocity to 0 after kick to prevent instability--
      walk.still=true;
      walk.set_velocity(0, 0, 0);
      return "done";
    end
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
end
