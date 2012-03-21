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
xTarget = Config.fsm.bodyApproach.xTarget0;
yTarget = Config.fsm.bodyApproach.yTarget0;

kick_type=1;

function set_approach_type(type)
  kick_type=type;

  --Check if walkkick is available
  if kick_type==2 then
    if walk.canWalkKick ~= 1 or Config.fsm.enable_walkkick == 0 then
      kick_type=1;
    end
  end

  if kick_type==1 then --Stationary Front kick
    xTarget = Config.fsm.bodyApproach.xTarget0;
    yTarget = Config.fsm.bodyApproach.yTarget0;
  elseif kick_type==2 then --Front walkkick
    xTarget = Config.fsm.bodyApproach.xTarget1;
    yTarget = Config.fsm.bodyApproach.yTarget1;
  end
end

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  ball = wcm.get_ball();
  yTarget1= sign(ball.y) * yTarget[2];
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
  vStep[1] = .6*(ball.x - xTarget[2]);
  vStep[2] = .75*(ball.y - yTarget1);
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  ballA = math.atan2(ball.y - math.max(math.min(ball.y, 0.05), -0.05),
            ball.x+0.10);
   vStep[3] = 0.5*ballA;--turn torwads the ball for approachSimple


--[[
  --turn towards the goal, not the ball  
  attackBearing, daPost = wcm.get_attack_bearing();
  if attackBearing > 10*math.pi/180 then
    vStep[3]=0.2;
  elseif attackBearing < -10*math.pi/180 then
    vStep[3]=-0.2;
  else
    vStep[3]=0;
  end
--]]
  --when the ball is on the side, backstep a bit
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

  if ((ball.x < xTarget[3]) and (math.abs(ball.y) < yTarget[3]) and
      (math.abs(ball.y) >= yTarget[1])) and (t-ball.t < 0.3) then
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
