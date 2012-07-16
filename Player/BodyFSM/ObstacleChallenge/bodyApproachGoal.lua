module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('Config')
require('wcm')
require('mcm')
require('ocm')

t0 = 0;
direction = 1;
timeout = 5
tLost = 0.5;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();

end

function update()
  local t = Body.get_time();

  attackBearing = wcm.get_attack_bearing();
  if attackBearing ~= nil then
    vx = 0.05;
    vy = 0;
    va = 0.2 * attackBearing;
    walk.set_velocity(vx, vy, va);
  end

  pose = wcm.get_robot_pose();
  if pose[1] > 3 then
    return "done"
  end

  if ocm.get_obstacle_front() == 1 then
    return "obstacle"
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
  mcm.set_walk_isSearching(0);
end
