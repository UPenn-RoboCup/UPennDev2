module(... or '', package.seeall)

require("Config")
require("vector")
require("util")
require("shm")

mapsize = Config.occ.mapsize;

shared = {}
shsize = {}

shared.occ = {};
shared.occ.reset = vector.zeros(1);
shared.occ.get_obstacle = vector.zeros(1);
shared.occ.map = 4 * mapsize * mapsize;
shared.occ.odom = vector.zeros(3);
shared.occ.vel = vector.zeros(3);

-- max 5 ob clusters
shared.ob = {};
shared.ob.num = vector.zeros(1);
shared.ob.centroid = vector.zeros(5 * 2);
shared.ob.angle_range = vector.zeros(5 * 2);
shared.ob.nearest = vector.zeros(5 * 3);


-- Robot Centroid x y on Map 
shared.occ.robot_pos = vector.zeros(2);

shsize.occ = shared.occ.map + 2^16;

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

function get_ob_x()
  obst = get_ob_nearest();
  nob = get_ob_num();
  x = {};
  idx = 1
  for i = 1 , nob * 3, 3 do
    x[idx] = obst[i];
    idx = idx + 1;
  end
  return x;
end

function get_ob_y()
  obst = get_ob_nearest();
  nob = get_ob_num();
  y = {};
  idx = 1
  for i = 2 , nob * 3, 3 do
    y[idx] = obst[i];
    idx = idx + 1;
  end
  return y;
end

function get_ob_dist()
  obst = get_ob_nearest();
  nob = get_ob_num();
  dist = {};
  idx = 1
  for i = 3 , nob * 3, 3 do
    dist[idx] = obst[i];
    idx = idx + 1;
  end
  return dist;
end

