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
shared.occ.map = 4 * mapsize * mapsize;
shared.occ.odom = vector.zeros(3);

-- Robot Centroid x y on Map 
shared.occ.robot_pos = vector.zeros(2);

shsize.occ = shared.occ.map + 2^16;

util.init_shm_segment(getfenv(), _NAME, shared, shsize);


