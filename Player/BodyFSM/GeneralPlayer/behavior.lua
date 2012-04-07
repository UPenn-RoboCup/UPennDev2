module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('Config')
require('wcm')

function update()
  --
  --Kick dir:1 front, 2 to the left, 3 to the right
  --Kick type: 1 stationary kick, 2 walkkick, (3 dribble)

  -------------------------------
  --Kickoff handling
  ------------------------------
  tKickOffWear = 20.0;

  t=Body.get_time();
  kick_off=wcm.get_kick_kickOff();
  tKickOff=wcm.get_kick_tKickOff();
  --If too long time has passed since game starts
  --Don't care about kickoff kick 
  if (t-tKickOff)>tKickOffWear then
    wcm.set_kick_kickOff(0);
    kick_off=0;
  end
  if kick_off>0 then 
    print("Behavior updated, kickoff kick")
    kickAngle = math.pi/6; --30 degree off angle
    kickDir=1;
    kickType=2;
    wcm.set_kick_kickOff(0);
    wcm.set_kick_dir(kickDir);
    wcm.set_kick_type(kickType);
    wcm.set_kick_angle(kickAngle);
    return;
  end

  attackBearing = wcm.get_attack_bearing();
  --Check if front walkkick is available now
  kickType=2;
  if walk.canWalkKick ~= 1 or Config.fsm.enable_walkkick == 0 then
    kickType=1;
  end

  --Check kick direction 
  thFrontKick = 45*math.pi/180;  

  if math.abs(attackBearing)<thFrontKick then
    kickDir=1;
    kickAngle = 0;
  elseif attackBearing>0 then --should kick to the left
    kickDir=2;
    kickAngle = math.pi/2;
  else
    kickDir=3;
    kickAngle = -math.pi/2;
  end

  if Config.fsm.enable_sidekick==0 then
    kickDir=1;
    kickAngle=0;
  end

  if kickDir==1 then
    print("Behavior updated, straight kick")
  elseif kickDir==2 then
    print("Behavior updated, kick to the left")
  else
    print("Behavior updated, kick to the right")
  end

  wcm.set_kick_dir(kickDir);
  wcm.set_kick_type(kickType);
  wcm.set_kick_angle(kickAngle);
end
