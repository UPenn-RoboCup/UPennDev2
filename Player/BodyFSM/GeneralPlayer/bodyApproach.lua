module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')

t0 = 0;
timeout = Config.fsm.bodyApproach.timeout;
maxStep = Config.fsm.bodyApproach.maxStep; -- maximum walk velocity
rFar = Config.fsm.bodyApproach.rFar;-- maximum ball distance threshold
tLost = Config.fsm.bodyApproach.tLost; --ball lost timeout

-- default kick threshold
xTarget = Config.fsm.bodyApproach.xTarget11;
yTarget = Config.fsm.bodyApproach.yTarget11;

--Do all kicks one by one
count=1;

function check_approach_type()

  kick_dir=wcm.get_kick_dir();
  kick_type=wcm.get_kick_type();
  kick_angle=wcm.get_kick_angle();

  print("Approach: kick dir /type ",kick_dir,kick_type)

  if kick_type==1 then --Stationary 
    if kick_dir==1 then --Front kick
      xTarget = Config.fsm.bodyApproach.xTarget11;
      yTarget = Config.fsm.bodyApproach.yTarget11;
    elseif kick_dir==2 then --Kick to left
      xTarget = Config.fsm.bodyApproach.xTarget12;
      yTarget = Config.fsm.bodyApproach.yTarget12;
    else
      xTarget = Config.fsm.bodyApproach.xTarget13;
      yTarget = Config.fsm.bodyApproach.yTarget13;
    end
  else --walkkick
    if kick_dir==1 then --Front kick
      xTarget = Config.fsm.bodyApproach.xTarget21;
      yTarget = Config.fsm.bodyApproach.yTarget21;
    elseif kick_dir==2 then --Kick to left
      xTarget = Config.fsm.bodyApproach.xTarget22;
      yTarget = Config.fsm.bodyApproach.yTarget22;
    else
      xTarget = Config.fsm.bodyApproach.xTarget23;
      yTarget = Config.fsm.bodyApproach.yTarget23;
    end
  end
end

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  ball = wcm.get_ball();
  check_approach_type(); --walkkick if available

print("Kick direction:",kick_dir)

  if kick_dir==1 then
    yTarget1= sign(ball.y) * yTarget[2];
  else
    yTarget1 = yTarget[2];
  end
  HeadFSM.sm:set_state('headTrack');
end

function update()
  local t = Body.get_time();

  -- get ball position
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  -- calculate walk velocity based on ball position
  vStep = vector.new({0,0,0});
  vStep[1] = .6*(ball.x - xTarget[2]);
  vStep[2] = .75*(ball.y - yTarget1);
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  ballA = math.atan2(ball.y - math.max(math.min(ball.y, 0.05), -0.05),
            ball.x+0.10);

  if Config.fsm.playMode==1 then 
    --Demo FSM, just turn towards the ball
    vStep[3] = 0.5*ballA;
  else
    --Player FSM, turn towards the goal
    attackBearing, daPost = wcm.get_attack_bearing();
    angle = util.mod_angle(attackBearing-kick_angle);
    if angle > 10*math.pi/180 then
      vStep[3]=0.2;
    elseif angle < -10*math.pi/180 then
      vStep[3]=-0.2;
    else
      vStep[3]=0;
    end
  end

  --when the ball is on the side, backstep a bit
  local wAngle = math.atan2 (vStep[2], vStep[1]);
  if math.abs(wAngle) > 70*math.pi/180 then
    vStep[1]=vStep[1] - 0.03;
  end
 
  walk.set_velocity(vStep[1],vStep[2],vStep[3]);

  if (t - ball.t > tLost) then
    print("ballLost")
    return "ballLost";
  end
  if (t - t0 > timeout) then
    print("timeout")
    return "timeout";
  end
  if (ballR > rFar) then
    print("ballfar, ",ballR,rFar)
    return "ballFar";
  end

--  print("Ball xy:",ball.x,ball.y);
--  print("Threshold xy:",xTarget[3],yTarget[3]);

  --TODO: angle threshold check
  if (ball.x < xTarget[3]) and (t-ball.t < 0.3) then
    if (kick_dir==1 and 
	(math.abs(ball.y) < yTarget[3]) and 
	(math.abs(ball.y) >= yTarget[1])) or
       (kick_dir~=1 and 
	(ball.y > yTarget[1]) and 
	(ball.y < yTarget[3])) 
    then
      if kick_type==1 then return "kick";
      elseif kick_type==2 then return "walkkick";
      end
    end
  end
end

function exit()
  HeadFSM.sm:set_state('headTrack');
end

function sign(x)
  if (x > 0) then return 1;
  elseif (x < 0) then return -1;
  else return 0;
  end
end
