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
  ballR = math.sqrt(ball.x^2+ ball.y^2);

  -- See where our home position is...
  homePosition=getGoalieHomePosition();
  homeRelative = util.pose_relative(homePosition, {pose.x, pose.y, pose.a});
  rHomeRelative = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);

  if ballR<rClose and t-ball.t<0.1 then
    Motion.event("walk");
    return "ballClose";
  end

  if ball.t<0.1 and ball.vx<-0.5 then
    dive.set_dive("diveLeft");
    Motion.event("dive");
    return "dive";
  end

  -- Check if out of position
  if( rHomeRelative>thFar ) then
    Motion.event("walk");
    return 'position';
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
  walk.start();
end
