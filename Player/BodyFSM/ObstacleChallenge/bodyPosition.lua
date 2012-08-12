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
--maxStep = 0.02

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

  attackBearing = wcm.get_attack_bearing();
  --recalculate approach path when ball is far away
  if ballR>0.60 then
    behavior.update();
  end

  role = gcm.get_team_role();
  kickDir = wcm.get_kick_dir();

  homePose = position.getAttackerHomePose();

  vx,vy,va=position.setAttackerVelocity(homePose);

  vel = ocm.get_occ_vel();
  vy = vy + 0.2 * vel[2];
  walk.set_velocity(vx,vy,va);
  
  if Config.fsm.avoidance_mode == 1 then 
    if ocm.get_obstacle_free() == 1 then
      return 'obstacle'
    end
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

end

function exit()
end

