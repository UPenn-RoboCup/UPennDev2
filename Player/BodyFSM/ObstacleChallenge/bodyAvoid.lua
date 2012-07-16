module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('util')
require('wcm')
require('gcm')
require('ocm')
--require('behaviorObstacle')

t0 = 0;
timeout = 5.0;

freeDir = 0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

--  Speak.talk('Avoiding Obstacle');

end

function update()
  local t = Body.get_time();
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  attackBearing, daPost = wcm.get_attack_bearing();
  attack_angle = util.mod_angle(attackBearing);

  left_obs = ocm.get_obstacle_left();
  right_obs = ocm.get_obstacle_right();
  if left_obs == 1 and right_obs == 1 then
    vStep = {-0.01, 0, 0}
  elseif left_obs == 1 then
    vStep = {0, -0.04, attackBearing - 25 * math.pi / 180}
  elseif right_obs == 1 then
    vStep = {0, 0.04, attackBearing + 25 * math.pi / 180}
  else
    if attack_angle > 10 * math.pi / 180 then
      vStep = {0, 0, 0.2}
    elseif attack_angle < -10 * math.pi / 180 then
      vStep = {0, 0, -0.2}
    else
      vStep = {0, 0, 0}
    end
  end
  walk.set_velocity(vStep[1], vStep[2], vStep[3]);
  
  front_obs = ocm.get_obstacle_front(); 
  if front_obs == 0 then
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

