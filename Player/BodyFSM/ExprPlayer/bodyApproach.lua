module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')
require('walk')

t0 = 0;
timeout = Config.fsm.bodyApproach.timeout;
maxStep = Config.fsm.bodyApproach.maxStep; -- maximum walk velocity
rFar = Config.fsm.bodyApproach.rFar;-- maximum ball distance threshold
tLost = Config.fsm.bodyApproach.tLost; --ball lost timeout

-- default kick threshold
xTarget = Config.fsm.bodyApproach.xTarget11;
yTarget = Config.fsm.bodyApproach.yTarget11;

fast_approach = Config.fsm.fast_approach or 0;

function check_approach_type()
  ball = wcm.get_ball();
  kick_dir=wcm.get_kick_dir();
  kick_type=wcm.get_kick_type();
  kick_angle=wcm.get_kick_angle();

  print("Approach: kick dir /type /angle",kick_dir,kick_type,kick_angle*180/math.pi)

  y_inv=0;
  if kick_type==1 then --Stationary 
    if kick_dir==1 then --Front kick
      xTarget = Config.fsm.bodyApproach.xTarget11;
      yTarget0 = Config.fsm.bodyApproach.yTarget11;
      if sign(ball.y)<0 then y_inv=1;end
    elseif kick_dir==2 then --Kick to the left
      xTarget = Config.fsm.bodyApproach.xTarget12;
      yTarget0 = Config.fsm.bodyApproach.yTarget12;
    else --Kick to the right
      xTarget = Config.fsm.bodyApproach.xTarget12;
      yTarget0 = Config.fsm.bodyApproach.yTarget12;
      y_inv=1;
    end
  else --walkkick
    if kick_dir==1 then --Front kick
      xTarget = Config.fsm.bodyApproach.xTarget21;
      yTarget0 = Config.fsm.bodyApproach.yTarget21;
      if sign(ball.y)<0 then y_inv=1; end
    elseif kick_dir==2 then --Kick to the left
      xTarget = Config.fsm.bodyApproach.xTarget22;
      yTarget0 = Config.fsm.bodyApproach.yTarget22;
    else --Kick to the right
      xTarget = Config.fsm.bodyApproach.xTarget22;
      yTarget0 = Config.fsm.bodyApproach.yTarget22;
      y_inv=1;
    end
  end

  if y_inv>0 then
    yTarget[1],yTarget[2],yTarget[3]=
      -yTarget0[3],-yTarget0[2],-yTarget0[1];
  else
     yTarget[1],yTarget[2],yTarget[3]=
       yTarget0[1],yTarget0[2],yTarget0[3];
  end

  print("Approach, target: ",xTarget[2],yTarget[2]);

end



function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  ball = wcm.get_ball();
  check_approach_type(); --walkkick if available

  if t0-ball.t<0.2 then
    ball_tracking=true;
    print("Ball Tracking")
    HeadFSM.sm:set_state('headKick');
  else
    ball_tracking=false;
  end
end

function update()
  local t = Body.get_time();
  -- get ball position 
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  if t-ball.t<0.2 and ball_tracking==false then
    ball_tracking=true;
    HeadFSM.sm:set_state('headKick');
  end


  --Current cordinate origin: midpoint of uLeft and uRight
  --Calculate ball position from future origin
  --Assuming we stop at next step
  if fast_approach ==1 then
    uLeft = walk.uLeft;
    uRight = walk.uRight;
    uFoot = util.se2_interpolate(0.5,uLeft,uRight); --Current origin 
    if walk.supportLeg ==0 then --left support 
      uRight2 = walk.uRight2;
      uLeft2 = util.pose_global({0,2*walk.footY,0},uRight2);
    else --Right support
      uLeft2 = walk.uLeft2;
      uRight2 = util.pose_global({0,-2*walk.footY,0},uLeft2);
    end
    uFoot2 = util.se2_interpolate(0.5,uLeft2,uRight2); --Projected origin 
    uMovement = util.pose_relative(uFoot2,uFoot);
    uBall2 = util.pose_relative({ball.x,ball.y,0},uMovement);
    ball.x=uBall2[1];
    ball.y=uBall2[2];
    factor_x = 0.8;
  else
    factor_x = 0.6;
  end

  
  -- calculate walk velocity based on ball position
  vStep = vector.new({0,0,0});
  vStep[1] = factor_x*(ball.x - xTarget[2]);
  vStep[2] = .75*(ball.y - yTarget[2]);
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  if Config.fsm.playMode==1 then 
    --Demo FSM, just turn towards the ball
    ballA = math.atan2(ball.y - math.max(math.min(ball.y, 0.05), -0.05),
            ball.x+0.10);
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
  if math.abs(wAngle) > 45*math.pi/180 then
    vStep[1]=vStep[1] - 0.03;
  else
    --Otherwise, don't make robot backstep
    vStep[1]=math.max(0,vStep[1]);
  end

  if walk.ph>0.95 then 
    print(string.format("Ball position: %.2f %.2f\n",ball.x,ball.y));
    print(string.format("Approach velocity:%.2f %.2f\n",vStep[1],vStep[2]));
  end

 
  walk.set_velocity(vStep[1],vStep[2],vStep[3]);

  if (t - ball.t > tLost) then
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

--  print("Ball xy:",ball.x,ball.y);
--  print("Threshold xy:",xTarget[3],yTarget[3]);

  --TODO: angle threshold check
  if (ball.x < xTarget[3]) and (t-ball.t < 0.5) and
     (ball.y > yTarget[1]) and (ball.y < yTarget[3]) then
    print(string.format("Approach done, ball position: %.2f %.2f\n",ball.x,ball.y))
    print(string.format("Ball target: %.2f %.2f\n",xTarget[2],yTarget[2]))
    if kick_type==1 then return "kick";
    else return "walkkick";
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
