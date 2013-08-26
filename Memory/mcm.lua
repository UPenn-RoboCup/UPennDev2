local memory = require'memory'

-- shared properties
local shared = {}
local shsize = {}

-- For the vision system
shared.camera = {}
shared.camera.bodyTilt = vector.zeros(1)
shared.camera.bodyHeight = vector.zeros(1)

shared.walk = {}
--are we on foot or on all fours?
shared.walk.bipedal    = vector.zeros(1)
shared.walk.bodyOffset = vector.zeros(3)
shared.walk.tStep      = vector.zeros(1)
shared.walk.bodyHeight = vector.zeros(1)
shared.walk.stepHeight = vector.zeros(1)
shared.walk.footY      = vector.zeros(1)
shared.walk.supportX   = vector.zeros(1)
shared.walk.supportY   = vector.zeros(1)
shared.walk.uLeft      = vector.zeros(3)
shared.walk.uRight     = vector.zeros(3)
shared.walk.vel        = vector.zeros(3)
shared.walk.current_vel        = vector.zeros(3)

--Robot specific calibration values
shared.walk.footXComp = vector.zeros(1)
shared.walk.kickXComp = vector.zeros(1)
shared.walk.headPitchBiasComp = vector.zeros(1)

-- How long have we been still for?
shared.walk.stillTime = vector.zeros(1)

-- Is the robot moving?
shared.walk.isMoving = vector.zeros(1)

--If the robot carries a ball, don't move arms
shared.walk.isCarrying = vector.zeros(1)
shared.walk.bodyCarryOffset = vector.zeros(3)

--To notify world to reset heading
shared.walk.isFallDown = vector.zeros(1)

--Is the robot spinning in bodySearch?
shared.walk.isSearching = vector.zeros(1)

shared.us = {}
shared.us.left      = vector.zeros(10)
shared.us.right     = vector.zeros(10)
shared.us.obstacles = vector.zeros(2)
shared.us.free      = vector.zeros(2)
shared.us.dSum      = vector.zeros(2)
shared.us.distance  = vector.zeros(2)

shared.motion = {}
--Should we perform fall check
shared.motion.fall_check = vector.zeros(1)

memory.init_shm_segment(..., shared, shsize)