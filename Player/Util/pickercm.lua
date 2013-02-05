module(..., package.seeall);

require('shm');
require('util');
require('vector');
require 'Config'

-- shared properties
shared = {};
shsize = {};

shared.desired = {};
shared.desired.rpy = vector.zeros(3);
shared.desired.velocity = vector.zeros(3);
shared.desired.pLArm = vector.zeros(4); -- Just Arm pos
shared.desired.pRArm = vector.zeros(4);
shared.desired.qLArm = vector.zeros(4); -- Just Arm pos
shared.desired.qRArm = vector.zeros(4);
shared.desired.dpLArm = vector.zeros(4); -- Change in Arm pos
shared.desired.dpRArm = vector.zeros(4);
shared.desired.bodyHeight = vector.zeros(3);
shared.desired.dbodyHeight = vector.zeros(3);

shared.device = {}
shared.device.t = vector.zeros(1);
shared.device.kind = vector.zeros(1);

util.init_shm_segment(getfenv(), 'pickercm', shared, shsize);
