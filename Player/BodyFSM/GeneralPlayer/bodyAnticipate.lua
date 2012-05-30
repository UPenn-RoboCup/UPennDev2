module(..., package.seeall);

require('Body')
require('vector')
require('Motion');
require('kick');
require('HeadFSM')
require('Config')
require('wcm')

require('walk');
require('dive')

t0 = 0;
tStart = 0;
timeout = 20.0;

started = false;
kickable = true;
follow = false;

maxPosition = 0.55;

tFollowDelay = Config.fsm.bodyKick.tFollowDelay;
rClose = Config.fsm.bodyAnticipate.rClose or 1.0;
thFar = Config.fsm.bodyAnticipate.thFar or 0.25;

goalie_dive = Config.goalie_dive or 0;

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  started = false;
  follow = false;
  Motion.event("diveready");
end

function update()
  local t = Body.get_time();
  ball = wcm.get_ball();
  pose = wcm.get_pose();
  tBall = Body.get_time() - ball.t;
  ballGlobal = util.pose_global({ball.x, ball.y, 0}, {pose.x, pose.y, pose.a});
  ballR = math.sqrt(ball.x^2+ ball.y^2);

  -- See where our home position is...
  homePosition=getGoalieHomePosition();
  homeRelative = util.pose_relative(homePosition, {pose.x, pose.y, pose.a});
  rHomeRelative = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);

--TODO: Diving handling 

  if goalie_dive > 0 then

    tStartDelay = 1.0;
--Penalty mark dist is 1.8m from goal line
    rCloseDive = 2.0; 
    rMinDive = 0.7;
    ball_velocity_th = -0.5;

    if t-t0>tStartDelay and t-ball.t<0.1 then
      --Tracking the ball in ready position. Stop off head movement
      --Body.set_head_hardness(0);
      ballR=math.sqrt(ball.x^2+ball.y^2);
      if ball.vx<ball_velocity_th and 
	ballR<rCloseDive and
	ballR>rMinDive then
        t0=t;
        py = ball.y - (ball.vy/ball.vx) * ball.x;
        print("Ball velocity:",ball.vx,ball.vy);
        print("Projected y pos:",py);
        if py>0.07 then 
          dive.set_dive("diveLeft");
        elseif py<-0.07 then
          dive.set_dive("diveRight");
        else
          dive.set_dive("diveCenter");
        end
        Motion.event("dive");
        return "dive";
      end
    end
  end

  if ballR<rClose and t-ball.t<0.1 then
    Motion.event("walk");
    return "ballClose";
  end
  -- Check if out of position
  if (t - t0 > timeout) and ( rHomeRelative>math.sqrt(thFar[1]^2+thFar[2]^2) ) then
    Motion.event("walk");
    return 'position';
  end
end
function getGoalieHomePosition()

  -- define home goalie position (in front of goal and facing the ball)
  --homePosition = 1.0*vector.new(wcm.get_goal_defend());
  homePosition = 0.98*vector.new(wcm.get_goal_defend());

  vBallHome = math.exp(-math.max(tBall-3.0, 0)/4.0)*(ballGlobal - homePosition);
  rBallHome = math.sqrt(vBallHome[1]^2 + vBallHome[2]^2);

  if (rBallHome > maxPosition) then
    scale = maxPosition/rBallHome;
    vBallHome = scale*vBallHome;
  end
  homePosition = homePosition + vBallHome;
  return homePosition;
end


function exit()
  walk.start();
end
