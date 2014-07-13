assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local world = {}

-------------------------------------------------------------
--GRASP LAB




-- Ball
world.ballDiameter = 0.22
-- Goal post


-------------------------------------------
-- Standard field

world.goalHeight = 1.8
world.goalWidth = 3
world.postDiameter = 0.1
world.obsDiameter = 0.2

-- Field
world.xBoundary = 4.5
world.yBoundary = 3.0
world.xMax = 4.8
world.yMax = 3.3
--Field corners
world.Lcorner={}
world.Lcorner[1]={4.5,3.0}
world.Lcorner[2]={4.5,-3.0}
world.Lcorner[3]={-4.5,3.0}
world.Lcorner[4]={-4.5,-3.0}
--Center T edge
world.Lcorner[5]={0,3.0}
world.Lcorner[6]={0,-3.0}

-------------------------------------------

-- Obstacle
world.obsDiameter = 0.15  -- our purchased ones
world.obsHeight = 0.9




-- M308 post
--[[
--------------------------------------------
world.goalHeight = 1.8
world.goalWidth = 2.6  -- in m308
------------------------------------------------
--]]


--GRASP small post
------------------------------------------------
---[[
world.goalHeight = 0.85
world.goalWidth = 1.6
--]]

if IS_WEBOTS then
  world.goalHeight = 1.8
  world.goalWidth = 3
end



world.goalUpper = {} -- Attacking goal
world.goalUpper[1] = {world.xBoundary, world.goalWidth/2}
world.goalUpper[2] = {world.xBoundary, -world.goalWidth/2}


world.goalLower = {}  -- Defending goal
world.goalLower[1] = {-world.xBoundary, -world.goalWidth/2}
world.goalLower[2] = {-world.xBoundary, world.goalWidth/2}

world.rGoalFilter = 0.02
world.aGoalFilter = 0.05
world.rPostFilter = 0.02
world.aPostFilter = 0.10
world.rUnknownPostFilter = 0.05
world.aUnknownPostFilter = 0.15
-- TODO
world.rLandmarkFilter = 0.05*3
world.aLandmarkFilter = 0.10*3
--
world.rCornerFilter = 0.01
world.aCornerFilter = 0.03
--
world.aLineFilter = 0.02

--New two-goalpost localization
world.use_new_goalposts=1

-- Pose estimation
world.nParticle = 100
world.initPosition = {0,0}
world.odomScale = {1,1,1} -- For now IMU not in use

-- Particle resampling intervals
world.resample_period = 0.3 -- seconds
world.resample_count = 20 -- partile filter cycles


world.use_imu_yaw = true

-- Associate with the table
Config.world = world

return Config
