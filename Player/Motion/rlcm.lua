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
--shared.params.bodyOffset = vector.zeros(3);

-- Start with default config values
shared.params.tStep = vector.ones(1) * Config.walk.tStep;
shared.params.bodyHeight = vector.ones(1) * Config.walk.bodyHeight;
shared.params.stepHeight = vector.zeros(1);
shared.params.footY = vector.zeros(1);
shared.params.supportX = vector.zeros(1);
shared.params.supportY = vector.zeros(1);

-- What is out trial information?
shared.trial = {};
shared.trial.num = vector.zeros(1);
shared.trial.stage = vector.ones(1);
shared.trial.reward = vector.zeros(nParams); -- Reward in each gradient step...

util.init_shm_segment(getfenv(), _NAME, shared, shsize);

-- Helper enumeration
local i = 0;
enum_param = {};
for param,value in pairs(shared.params) do
  i = i+1;
  enum_param[i] = param;
end 
