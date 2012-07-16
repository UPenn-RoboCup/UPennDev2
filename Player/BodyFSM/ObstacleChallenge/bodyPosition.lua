module(..., package.seeall);

require('Body')
require('World')
require('walk')
require('vector')
require('wcm')
require('ocm')
require('Config')
require('Team')
require('util')
require('walk')

require('behavior')
require('position')

t0 = 0;

tLost = Config.fsm.bodyPosition.tLost;
timeout = Config.fsm.bodyPosition.timeout;
thClose = Config.fsm.bodyPosition.thClose;
rClose= Config.fsm.bodyPosition.rClose;
fast_approach=Config.fsm.fast_approach or 0;

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  max_speed=0;
  count=0;
  ball=wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);
  maxStep=maxStep1;
  behavior.update();
end


function update()
  count=count+1;

  local t = Body.get_time();
  ball=wcm.get_ball();
  pose=wcm.get_pose();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  --recalculate approach path when ball is far away
  if ballR>0.60 then
    behavior.update();
  end

  role = gcm.get_team_role();
  kickDir = wcm.get_kick_dir();

  homePose = position.getAttackerHomePose();

  vx,vy,va=position.setAttackerVelocity(homePose);

  walk.set_velocity(vx,vy,va);

  if ocm.get_obstacle_front() == 1 then
    return 'obstacle'
  end

  if (t - ball.t > tLost) then
    return "ballLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end

  tBall=0.5;

  if ballR<rClose then
    print("bodyPosition ballClose")
    return "ballClose";
  end

  attackAngle = wcm.get_goal_attack_angle2();
  daPost = wcm.get_goal_daPost2();
  daPostMargin = 15 * math.pi/180;
  daPost1 = math.max(thClose[3],daPost/2 - daPostMargin);

  uPose=vector.new({pose.x,pose.y,pose.a})
  homeRelative = util.pose_relative(homePose, uPose);  
  angleToTurn = math.max(0, homeRelative[3] - daPost1);

  if math.abs(homeRelative[1])<thClose[1] and
    math.abs(homeRelative[2])<thClose[2] and
    math.abs(homeRelative[3])<daPost1 and
    ballR<rClose and
    t-ball.t<tBall then
      print("bodyPosition done")
      return "done";
  end
end

function exit()
end

