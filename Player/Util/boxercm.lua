module(..., package.seeall);

require('shm');
require('util');
require('vector');
require 'Config'

-- shared properties
shared = {};
shsize = {};

-- Add the body information
shared.body.enabled = vector.zeros(1);
shared.body.rpy = vector.zeros(3);
shared.body.velocity = vector.zeros(3);
shared.body.punch = vector.zeros(2); -- For both hands
shared.body.qLArm = vector.zeros(3);
shared.body.qRArm = vector.zeros(3);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

