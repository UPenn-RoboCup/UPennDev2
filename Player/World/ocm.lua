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
shared.ob.num = vector.zeros(1);
shared.ob.centroid = vector.zeros(5 * 2);
shared.ob.angle_range = vector.zeros(5 * 2);
shared.ob.nearest = vector.zeros(5 * 3);


-- Robot Centroid x y on Map 
shared.occ.robot_pos = vector.zeros(2);

shsize.occ = shared.occ.map + 2^16;

util.init_shm_segment(getfenv(), _NAME, shared, shsize);


