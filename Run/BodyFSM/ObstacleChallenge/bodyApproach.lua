module(..., package.seeall);

require('Body')
require('wcm')
require('ocm')
require('walk')
require('vector')
require('walk')
require('position')

t0 = 0;
timeout = Config.fsm.bodyApproach.timeout;
maxStep = Config.fsm.bodyApproach.maxStep; -- maximum walk velocity
rFar = Config.fsm.bodyApproach.rFar;-- maximum ball distance threshold
tLost = Config.fsm.bodyApproach.tLost; --ball lost timeout

-- default kick threshold
xTarget = Config.fsm.bodyApproach.xTarget11;
yTarget = Config.fsm.bodyApproach.yTarget11;

dapost_check = Config.fsm.daPost_check or 0;
daPostMargin = Config.fsm.daPostMargin or 15*math.pi/180;

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  ball = wcm.get_ball();

  if t0-ball.t<0.2 then
    ball_tracking=true;
    print("Ball Tracking")
    HeadFSM.sm:set_state('headTrack');
  else
    ball_tracking=false;
  end

  role = gcm.get_team_role();
end

function update()
  local t = Body.get_time();
  -- get ball position 
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  if t-ball.t<0.2 and ball_tracking==false then
    ball_tracking=true;
    HeadFSM.sm:set_state('headTrack');
  end

  factor_x = 0.6;
  
  -- calculate walk velocity based on ball position
  vStep = vector.new({0,0,0});
  vStep[1] = factor_x*(ball.x - xTarget[2]);
  vStep[2] = .75*(ball.y - yTarget[2]);
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  --Player FSM, turn towards the goal
  --attackBearing, daPost = wcm.get_attack_bearing();
  position.posCalc();

  kickAngle = wcm.get_kick_angle();
  attackAngle = wcm.get_goal_attack_angle2()-kickAngle;
  daPost = wcm.get_goal_daPost2();

  pose=wcm.get_pose();

  --when the ball is on the side of the ROBOT, backstep a bit
  local wAngle = math.atan2 (ball.y,ball.x);

  ballYMin = Config.fsm.bodyApproach.ballYMin or 0.20;

  if math.abs(wAngle) > 45*math.pi/180 then
    vStep[1]=vStep[1] - 0.03;

    if ball.y<ballYMin and ball.y>0 then
     vStep[2] = -0.03;
    elseif ball.y<0 and ball.y>-ballYMin then
      vStep[2]=0.03;
    else
      vStep[2] = 0;
    end    

  else
    --Otherwise, don't make robot backstep
    vStep[1]=math.max(0,vStep[1]);
  end

--  if walk.ph>0.95 then 
--    print(string.format("Ball position: %.2f %.2f\n",ball.x,ball.y));
--    print(string.format("Approach velocity:%.2f %.2f\n",vStep[1],vStep[2]));
--  end

  if Config.fsm.avoidance_mode == 1 then
    if ocm.get_obstacle_free() == 1 then 
      return 'obstacle';
    end
  end
 
  vel = ocm.get_occ_vel();
  vStep[2] = vStep[2] + 0.2 * vel[2];
  walk.set_velocity(vStep[1],vStep[2],vStep[3]);
  
  pose = wcm.get_robot_pose()
  if pose[1] > 3 then 
    return 'done'
  end

  if (t - ball.t > tLost) and role>0 then
    HeadFSM.sm:set_state('headScan');
    print("ballLost")
    return "ballLost";
  end
  if (t - t0 > timeout) then
    HeadFSM.sm:set_state('headTrack');
    print("timeout")
    return "timeout";
  end
  if (ballR > rFar) then
    HeadFSM.sm:set_state('headTrack');
    print("ballfar, ",ballR,rFar)
    return "ballFar";
  end

  --For front kick, check for other side too
    yTargetMin = math.min(math.abs(yTarget[1]),math.abs(yTarget[3]));
    yTargetMax = math.max(math.abs(yTarget[1]),math.abs(yTarget[3]));

    if (ball.x < xTarget[3]) and (t-ball.t < 0.5) then --and
--       (math.abs(ball.y) > yTargetMin) and (math.abs(ball.y) < yTargetMax) and angle_check_done then
      print(string.format("Approach done, ball position: %.2f %.2f\n",ball.x,ball.y))
      print(string.format("Ball target: %.2f %.2f\n",xTarget[2],yTarget[2]))
      return "kick";
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
