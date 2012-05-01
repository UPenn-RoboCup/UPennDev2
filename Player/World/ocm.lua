module(... or '', package.seeall)

require("Config")
require("vector")
require("util")
require("shm")

mapsize = 50;

shared = {}
shsize = {}

shared.occ = {};
shared.occ.map = vector.zeros(mapsize * mapsize);

shared.centroid = {};
shared.centroid.x = vector.zeros(1);
shared.centroid.y = vector.zeros(1);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);


