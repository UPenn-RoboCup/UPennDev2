module(..., package.seeall);

require('shm');
require('util');
require('vector');
require 'Config'

-- shared properties
shared = {};
shsize = {};

-- Add the body information
shared.body = {};
shared.body.enabled = vector.zeros(1);
shared.body.rpy = vector.zeros(3);
shared.body.velocity = vector.zeros(3);
shared.body.punchL = vector.zeros(1);
shared.body.punchR = vector.zeros(1);
shared.body.qLArm = vector.zeros(3);
shared.body.qRArm = vector.zeros(3);

-- Add debugging of the state machine
shared.fsm = {}
shared.fsm.state = '';

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

