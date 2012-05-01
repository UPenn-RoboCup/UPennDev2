module(... or '', package.seeall)

require("Config")
require("vector")
require("util")
require("shm")

mapsize = Config.occ.mapsize;

shared = {}
shsize = {}

shared.occ = {};
shared.occ.map = vector.zeros(mapsize * mapsize);

-- Robot Centroid x y on Map 
shared.occ.centroid = vector.zeros(2);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);


