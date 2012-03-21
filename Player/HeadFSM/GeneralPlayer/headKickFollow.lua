------------------------------
-- Follow the ball after kicking
------------------------------

module(..., package.seeall);

require('Body')
require('wcm')
require('mcm')

t0 = 0;

-- follow period
tFollow = Config.fsm.headKickFollow.tFollow;
pitch0=Config.fsm.headKickFollow.pitch[1];
pitch1=Config.fsm.headKickFollow.pitch[2];
pitchSide=Config.fsm.headKickFollow.pitchSide;
yawMagSide=Config.fsm.headKickFollow.yawMagSide;

function entry()
  print("Head SM:".._NAME.." entry");

  t0 = Body.get_time();
-- TODO: side kick
-- kickType = wcm.get_kick_type();
  kickType = 0; --straight kick
end

function update()
  local t = Body.get_time();
  local ph = (t-t0)/tFollow;

  if kickType == 0 then --front kick
      pitch = (1-ph)*pitch0 + ph*pitch1;
      yaw=0;
  elseif vcm.kickdir==1 then --sidekick left
      pitch = (1-ph)*pitch0 + ph*pitch1;
      yaw = ph*-yawMagSide;
  else --sidekick right
      pitch = (1-ph)*pitch0 + ph*pitch1;
      yaw = ph*yawMagSie;
  end
  local pitch_actual = pitch - Config.head.cameraAngle[1][2];
  Body.set_head_command({yaw, pitch_actual});

  local ball = wcm.get_ball();
  if (t - ball.t < 0.1) then
    print("BallFound")
    return "ball";
  end
  if (t - t0 > tFollow) then
    return "lost";
  end
end

function exit()
end
