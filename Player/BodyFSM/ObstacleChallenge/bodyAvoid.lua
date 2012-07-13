module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('util')
require('wcm')
require('gcm')
require('behaviorObstacle')

t0 = 0;
timeout = 5.0;

freeDir = 0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

  Speak.talk('Avoiding Obstacle');
  obs = behaviorObstacle.check_obstacle({0, 0, 0})
  if obs.left and obs.right then
    freeDir = 1 -- both size occupied, need slow down and backstep
  elseif obs.left then
    freeDir = 2 -- right side free
  elseif obs.right then
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
    vStep = {-0.02, 0, 0}
  elseif freeDir == 2 then
    vStep = {0, 0, -0.02}
  elseif freeDir == 3 then
    vStep = {0, 0, 0.02}
  else
    if angle > 10 * math.pi / 180 then
      vStep = {0, 0, 0.2}
    elseif angle < -10 * math.pi / 180 then
      vStep = {0, 0, -0.2}
    else
      vStep = {0, 0, 0}
    end
  end
  walk.set_velocity(vStep[1], vStep[2], vStep[3]);

  
  obs = behaviorObstacle.check_obstacle({0,0,0})
  
  if obs.front then
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

