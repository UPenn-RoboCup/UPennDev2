module(..., package.seeall);

require('shm');
require('util');
require('vector');
require 'Config'

-- shared properties
shared = {};
shsize = {};

shared.skeleton = {};
shared.skeleton.torso = vector.zeros(3);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

