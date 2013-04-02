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
shared.body.t = vector.zeros(1);
shared.body.enabled = vector.zeros(1);
shared.body.rpy = vector.zeros(3);
shared.body.velocity = vector.zeros(3);
shared.body.punchL = vector.zeros(1);
shared.body.punchR = vector.zeros(1);
shared.body.qLArm = vector.zeros(3);
shared.body.qRArm = vector.zeros(3);

-- Add debugging of the state machine
shared.fsm = {}
shared.fsm.stateL = '';
shared.fsm.stateR = '';

print('Init shm for ',_NAME)

-- Check if a number given
-- This number accesses a playerID of that number
if(string.len(_NAME)>7) then
  pid = string.sub(_NAME,8);
end

if( pid ) then
  util.init_shm_segment(getfenv(), 'boxercm', shared, shsize,nil,pid);
else
  util.init_shm_segment(getfenv(), 'boxercm', shared, shsize);
end
