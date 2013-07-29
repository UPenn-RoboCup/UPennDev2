local shm = require('shm');
local util = require('util');
local vector = require('vector');
local Config = require('Config');

-- shared properties
local shared = {}
local shsize = {}

-- For the vision system
shared.camera = {}
shared.camera.bodyTilt = vector.zeros(1);
shared.camera.bodyHeight = vector.zeros(1);

shared.walk = {};
shared.walk.bipedal = vector.zeros(1); --are we on foot or on four?

shared.walk.bodyOffset = vector.zeros(3);
shared.walk.tStep = vector.zeros(1);
shared.walk.bodyHeight = vector.zeros(1);
shared.walk.stepHeight = vector.zeros(1);
shared.walk.footY = vector.zeros(1);
shared.walk.supportX = vector.zeros(1);
shared.walk.supportY = vector.zeros(1);
shared.walk.uLeft = vector.zeros(3);
shared.walk.uRight = vector.zeros(3);
shared.walk.vel = vector.zeros(3);

--Robot specific calibration values
shared.walk.footXComp = vector.zeros(1);
shared.walk.kickXComp = vector.zeros(1);
shared.walk.headPitchBiasComp = vector.zeros(1);

-- How long have we been still for?
shared.walk.stillTime = vector.zeros(1);

-- Is the robot moving?
shared.walk.isMoving = vector.zeros(1);

--If the robot carries a ball, don't move arms
shared.walk.isCarrying = vector.zeros(1);
shared.walk.bodyCarryOffset = vector.zeros(3);

--To notify world to reset heading
shared.walk.isFallDown = vector.zeros(1);

--Is the robot spinning in bodySearch?
shared.walk.isSearching = vector.zeros(1);

shared.us = {};
shared.us.left = vector.zeros(10);
shared.us.right = vector.zeros(10);
shared.us.obstacles = vector.zeros(2);
shared.us.free = vector.zeros(2);
shared.us.dSum = vector.zeros(2);
shared.us.distance = vector.zeros(2);

shared.motion = {};
--Should we perform fall check
shared.motion.fall_check = vector.zeros(1);

util.init_shm_segment(..., shared, shsize);

-- helper functions

---
--Get the distance moved in one step
--@param u0 The previous position
--@return The Distance moved with the current walk plan
--@return The global position of the planned step
function mcm.get_odometry(u0)
  if (not u0) then
    u0 = vector.new({0, 0, 0});
  end

  local uFoot = util.se2_interpolate(.5, mcm.get_walk_uLeft(), mcm.get_walk_uRight());
  return util.pose_relative(uFoot, u0), uFoot;
end

--Now those parameters are dynamically adjustable
local footX = Config.walk.footX or 0;
local kickX = Config.walk.kickX or 0;
local footXComp = Config.walk.footXComp or 0;
local kickXComp = Config.walk.kickXComp or 0;
local headPitchBias = Config.walk.headPitchBias or 0;
local headPitchBiasComp = Config.walk.headPitchBiasComp or 0;

mcm.set_walk_footXComp(footXComp);
mcm.set_walk_kickXComp(kickXComp);
mcm.set_walk_headPitchBiasComp(headPitchBiasComp);

function mcm.get_footX()
  return mcm.get_walk_footXComp() + footX;
end

function mcm.get_kickX()
  return mcm.get_walk_kickXComp();
end

function mcm.get_headPitchBias()
  return mcm.get_walk_headPitchBiasComp()+headPitchBias;
end

