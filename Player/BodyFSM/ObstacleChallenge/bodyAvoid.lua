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
tLost = 0.5;

xTarget = Config.fsm.bodyApproach.xTarget11;
yTarget = Config.fsm.bodyApproach.yTarget11;

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
  front_obs = ocm.get_obstacle_front();
  if front_obs == 0 then
    vStep[1] = 0.03;
    vStep[2] = 0;
    vStep[3] = 0.2 * attackBearing;
  else
    if left_obs == 1 and right_obs == 1 then 
      vStep[1] = -0.005 
--      vStep[1] = 0.0 
      vStep[3] = 0.2 * attackBearing;
    elseif left_obs == 1 then 
  --    vStep[1] = -0.02 
      vStep[1] = 0.0 
      vStep[2] = -0.02
      vStep[3] = 0.2 * attackBearing - 5 * math.pi / 180 
    elseif right_obs == 1 then
  --    vStep[1] = -0.02
      vStep[1] = 0.0
      vStep[2] = 0.02
      vStep[3] = 0.2 * attackBearing + 5 * math.pi / 180
    else
      vStep[1] = 0.0
      if attackBearing > 0 then
        vStep[2] = 0.02
      else
        vStep[2] = -0.02
      end
  --    vStep[1] = -0.04
--      if attack_angle > 10 * math.pi / 180 then
--        vStep[3] = 0.1 * attackBearing + 5 * math.pi / 180
--      elseif attack_angle < -10 * math.pi / 180 then
--        vStep[3] = 0.1 * attackBearing - 5 * math.pi / 180
--      else
      vStep[3] = 0.2 * attackBearing;
--      end
    end
  end
  -- if dribble mode, calculate velocity based on ball
  if Config.fsm.avoidance_mode == 1 and 
    -- when tracking the ball
    (t - ball.t < tLost )  then
--    print('Avoidance Velocity calc from ball')
    vStep[1] = 0.5 * (ball.x + 1.5 * vStep[1] + 1.0 * xTarget[2]); 
    vStep[2] = 0.7 * (ball.y + 1.5 * vStep[2] + 1.0 * yTarget[2]);
    vStep[3] = 0;
    if right_obs == 1 then 
      vStep[3] = attackBearing -- - 55 * math.pi / 180;
    elseif left_obs == 1 then
      vStep[3] = attackBearing -- + 55 * math.pi / 180;
    else
      vStep[3] = attackBearing --0 * math.pi / 180;
    end
--    print(vStep[1], vStep[2], vStep[3])
    scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
--    print(scale)
    vStep = scale*vStep;
  end

--  if vcm.get_freespace_allBlocked() == 1 then
--    print('BodyAvoid: view blocked, STOP!!!')
--    vStep[1] = -0.005;
--  end

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

