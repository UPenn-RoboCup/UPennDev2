module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('Config')
require('wcm')

function update()
  --
  --Kick dir:1 front, 2 to the left, 3 to the right
  --Kick type: 1 stationary kick, 2 walkkick

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
    kickType=1;
  else
    print("Behavior updated, kick to the right")
    kickType=1;
  end

  wcm.set_kick_dir(kickDir);
  wcm.set_kick_type(kickType);
  wcm.set_kick_angle(kickAngle);
end
