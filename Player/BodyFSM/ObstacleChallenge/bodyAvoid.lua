module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('util')
require('wcm')
require('gcm')
require('ocm')
require('behaviorObstacle')

t0 = 0;
timeout = 5.0;

freeDir = 0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

  Speak.talk('Avoiding Obstacle');
  left_free = ocm.get_obstacle_left();
  right_free = ocm.get_obstacle_right();

  if left_free == 1 and right_free == 1 then
    freeDir = 1 -- both size occupied, need slow down and backstep
  elseif left_free == 1 then
    freeDir = 2 -- right side free
  elseif right_free == 1 then
    freeDir = 3 -- left side free
  else
    freeDir = 0 -- both size free
  end

end

function update()
  local t = Body.get_time();
  ball = wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  attackBearing, daPost = wcm.get_attack_bearing();
  attack_angle = util.mod_angle(attackBearing);

  if freeDir == 1 then
    vStep = {-0.01, 0, 0}
  elseif freeDir == 2 then
    vStep = {0, 0, -0.02}
  elseif freeDir == 3 then
    vStep = {0, 0, 0.02}
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
  
  front_free = ocm.get_obstacle_front(); 
  if front_free == 1 then
    return "obstacle"
  else
    return "clear"
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
  walk.start();
end

