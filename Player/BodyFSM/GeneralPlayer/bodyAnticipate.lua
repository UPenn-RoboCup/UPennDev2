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
timeout = 30.0;

started = false;
kickable = true;
follow = false;

maxPosition = 0.55;

tFollowDelay = Config.fsm.bodyKick.tFollowDelay;
rClose = Config.fsm.bodyAnticipate.rClose or 1.0;
thFar = Config.fsm.bodyAnticipate.thFar or 0.25;

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

  if ballR<rClose and t-ball.t<0.1 then
    Motion.event("walk");
    return "ballClose";
  end

--[[
  if ball.t<0.1 and ball.vx<-0.5 then
    dive.set_dive("diveLeft");
    Motion.event("dive");
    return "dive";
  end
--]]

  -- Check if out of position
  if( rHomeRelative>math.sqrt(thFar[1]^2+thFar[2]^2) ) then
    Motion.event("walk");
    return 'position';
  end

  if (t - t0 > timeout) then
    return "timeout";
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
