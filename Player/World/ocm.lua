module(... or '', package.seeall)

require("Config")
require("vector")
require("util")
require("shm")

mapsize = Config.occ.mapsize;

shared = {}
shsize = {}

shared.occ = {};
shared.occ.map = 4 * mapsize * mapsize;
shared.occ.odom = vector.zeros(3);
shared.occ.vel = vector.zeros(3);

-- max 5 obstacle clusters
shared.obstacle = {};
shared.obstacle.num = vector.zeros(1);
-- centroids
shared.obstacle.cx = vector.zeros(5);
shared.obstacle.cy = vector.zeros(5);
-- range in terms of angle
shared.obstacle.la = vector.zeros(5);
shared.obstacle.ra = vector.zeros(5);
-- nearest point
shared.obstacle.nx = vector.zeros(5);
shared.obstacle.ny = vector.zeros(5);
-- nearest distance
shared.obstacle.ndist = vector.zeros(5);
-- general flag for occupied direction
shared.obstacle.front = vector.zeros(1);
shared.obstacle.left = vector.zeros(1);
shared.obstacle.right = vector.zeros(1);
shared.obstacle.free = vector.zeros(1);

-- Robot Centroid x y on Map 
shared.occ.robot_pos = vector.zeros(2);

shsize.occ = shared.occ.map + 2^16;

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

function get_obstacle_x()
  obst = get_obstacle_nearest();
  nob = get_obstacle_num();
  x = {};
  idx = 1
  for i = 1 , nob * 3, 3 do
    x[idx] = obst[i];
    idx = idx + 1;
  end
  return x;
end

function get_obstacle_y()
  obst = get_obstacle_nearest();
  nob = get_obstacle_num();
  y = {};
  idx = 1
  for i = 2 , nob * 3, 3 do
    y[idx] = obst[i];
    idx = idx + 1;
  end
  return y;
end

function get_obstacle_dist()
  obst = get_obstacle_nearest();
  nob = get_obstacle_num();
  dist = {};
  idx = 1
  for i = 3 , nob * 3, 3 do
    dist[idx] = obst[i];
    idx = idx + 1;
  end
  return dist;
end

