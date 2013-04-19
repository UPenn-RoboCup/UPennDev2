module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')
local vector = require('vector')
local Config = require('Config')
local wcm = require('wcm')
local mcm = require('mcm')
local ocm = require('ocm')

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
  if Config.fsm.avoidance_mode == 2 then
    vel = ocm.get_occ_vel();
    walk.set_velocity(vel[1], vel[2], 0.2 * attackBearing);
  else
    if attackBearing ~= nil then
      vx = 0.03;
      vy = 0;
      va = 0.2 * attackBearing;
      walk.set_velocity(vx, vy, va);
    end
    if ocm.get_obstacle_free() == 1 then
      return "obstacle"
    end
  end

  pose = wcm.get_robot_pose();
  if pose[1] > 3 then
    return "done"
  end

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
  mcm.set_walk_isSearching(0);
end
