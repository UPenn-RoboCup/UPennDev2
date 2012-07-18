module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('util')
require('wcm')
require('gcm')
require('ocm')

t0 = 0;
timeout = 5.0;
maxStep = 0.03;
freeDir = 0;
tLost = 1;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

--  Speak.talk('Avoiding Obstacle');

end

function update()
  local t = Body.get_time();
  vStep = vector.new({0, 0, 0})
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  attackBearing, daPost = wcm.get_attack_bearing();
  attack_angle = util.mod_angle(attackBearing);

  left_obs = ocm.get_obstacle_left();
  right_obs = ocm.get_obstacle_right();
  if left_obs == 1 and right_obs == 1 then
    vStep[1] = -0.02
  elseif left_obs == 1 then
    vStep[1] = -0.02
    vStep[2] = -0.04
    vStep[3] = attackBearing - 25 * math.pi / 180
  elseif right_obs == 1 then
    vStep[1] = -0.02
    vStep[2] = 0.04
    vStep[3] = attackBearing + 25 * math.pi / 180
  else
    vStep[1] = -0.04
    if attack_angle > 10 * math.pi / 180 then
      vStep[3] = attack_angle + 25 * math.pi / 180
    elseif attack_angle < -10 * math.pi / 180 then
      vStep[3] = attack_angle - 25 * math.pi / 180
    end
  end
  
  -- if dribble mode, calculate velocity based on ball
  if Config.fsm.avoidance_mode == 1 and 
    -- when tracking the ball
    (t - ball.t < tLost )  then
    vStep[1] = .6*(ball.x - vStep[1]);
    vStep[2] = .5*(ball.y - vStep[2]);
    scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
    vStep = scale*vStep;
  end

  walk.set_velocity(vStep[1], vStep[2], vStep[3]);
  
  if ocm.get_obstacle_free() == 0 then
    print('Avoided Front obstacle')
    return "done"
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
  walk.start();
end

