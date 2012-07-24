module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('util')
require('vector')

t0 = 0;
timeout = 10.0;

-- maximum walk velocity
maxStep = 0.025;

-- ball detection timeout
tLost = 3.0;

-- Grip threshold
xTarget = Config.fsm.bodyApproach.xTargetGrip or {0.13,0.15,0.16};
yTarget0 = Config.fsm.bodyApproach.yTargetGrip or {0.05,0.07,0.09};


maxStep = 0.02;



-- maximum ball distance threshold
rFar = 0.45;

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  ball = wcm.get_ball();


xTarget = {0.02,0.03,0.04};
yTarget = {0.10,0.11,0.12};

end

function update()
  local t = Body.get_time();

  -- get ball position
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  -- calculate walk velocity based on ball position
  vStep = vector.new({0,0,0});
  vStep[1] = .6*(ball.x - xTarget[2]);
  vStep[2] = .75*(ball.y - yTarget[2]);
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  vStep[3]=0; --Don't turn 
  --when the ball is on the side, backstep a bit
  --[[
  local wAngle = math.atan2 (vStep[2], vStep[1]);
  if math.abs(wAngle) > 70*math.pi/180 then
    vStep[1]=vStep[1] - 0.03;
  end
  --]]
 
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

  if (ball.x < xTarget[3]) and (t-ball.t < 0.3) and
     (ball.y > yTarget[1]) and (ball.y < yTarget[3]) then
     return "pickup";
  end
end

function exit()
end
