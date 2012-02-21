module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')

t0 = 0;
timeout = 10.0;

-- maximum walk velocity
maxStep = 0.03;


-- kick threshold
yKickMin = 0.01;
yKickMax = 0.05;
yTarget0 = 0.04;

yKickMin = 0.02;

-- maximum ball distance threshold
rFar = 0.45;

-- ball detection timeout
tLost = 3.0*Config.speedFactor;


kick_type=1;

function set_approach_type(type)
  kick_type=type;

  --Check if walkkick is available
  if kick_type==2 and walk.canWalkKick ~= 1 then
    kick_type=1;
  end
--  kick_type=1; --hack


  if kick_type==1 then --Stationary Front kick
    xKick = 0.14;
    xTarget = 0.13;
  elseif kick_type==2 then --Front walkkick
    xKick = 0.20;
    xTarget = 0.17;

--webots parameters
yKickMin = 0.03;
yTarget0 = 0.05;
yKickMax = 0.06;


  end
end

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  ball = wcm.get_ball();
  yTarget= sign(ball.y) * yTarget0;
  HeadFSM.sm:set_state('headTrack');
 
  set_approach_type(2); --walkkick if available
end

function update()
  local t = Body.get_time();

  -- get ball position
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  -- calculate walk velocity based on ball position
  vStep = vector.new({0,0,0});
  vStep[1] = .6*(ball.x - xTarget);
  vStep[2] = .75*(ball.y - yTarget);
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  ballA = math.atan2(ball.y - math.max(math.min(ball.y, 0.05), -0.05),
            ball.x+0.10);

  --  vStep[3] = 0.5*ballA;
  --SJ: turn towards the goal, not the ball  
  attackBearing, daPost = wcm.get_attack_bearing();
  if attackBearing > 10*math.pi/180 then
    vStep[3]=0.2;
  elseif attackBearing < -10*math.pi/180 then
    vStep[3]=-0.2;
  else
    vStep[3]=0;
  end

  --SJ: when the ball is on the side, backstep a bit
  local wAngle = math.atan2 (vStep[2], vStep[1]);
  if math.abs(wAngle) > 70*math.pi/180 then
    vStep[1]=vStep[1] - 0.03;
  end
 
  walk.set_velocity(vStep[1],vStep[2],vStep[3]);


  if (t - ball.t > tLost) then
    return "ballLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end
  if (ballR > rFar) then
    return "ballFar";
  end

  if ((ball.x < xKick) and (math.abs(ball.y) < yKickMax) and
      (math.abs(ball.y) > yKickMin)) then
    if kick_type==1 then 
      return "kick";
    elseif kick_type==2 then
print("WALKKICK")
      return "walkkick";
    end
  end
  if (t - t0 > 1.0 and Body.get_sensor_button()[1] > 0) then
    return "button";
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
