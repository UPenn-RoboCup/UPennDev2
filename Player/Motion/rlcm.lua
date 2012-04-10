module(..., package.seeall);

require('shm');
require('util');
require('vector');
require('Config');

-- shared properties
shared = {};
shsize = {};

-- What are our parameters?
shared.params = {};
shared.params.bodyOffset = vector.zeros(3);
shared.params.tStep = vector.zeros(1);
shared.params.bodyHeight = vector.zeros(1);
shared.params.stepHeight = vector.zeros(1);
shared.params.footY = vector.zeros(1);
shared.params.supportX = vector.zeros(1);
shared.params.supportY = vector.zeros(1);
shared.params.uLeft = vector.zeros(3);
shared.params.uRight = vector.zeros(3);

-- What is out trial information?
shared.trial = {};
shared.trial.num = vector.zeros(1);
shared.trial.reward = vector.zeros(1);

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

