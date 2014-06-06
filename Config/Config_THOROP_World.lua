assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local world = {}

-- Ball
world.ballDiameter = 0.22
-- Goal post
world.postDiameter = 0.1
world.goalHeight = 1.8
world.goalWidth = 3
world.goalUpper = {} -- Goal in use
world.goalUpper[1] = {4.5, 1.5}
world.goalUpper[2] = {4.5, -1.5}
world.goalLower = {}  -- Goal not in use
world.goalLower[1] = {-4.5, -1.5}
world.goalLower[2] = {-4.5, 1.5}

-- Obstacle
world.obsDiameter = 0.2
world.obsHeight = 0.9

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



-- Filter weights
world.rGoalFilter = 0.02*3
world.aGoalFilter = 0.05*3
world.rPostFilter = 0.02*3
world.aPostFilter = 0.10*3
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
world.use_imu_yaw = false
-- Particle resampling intervals
world.resample_period = 0.3 -- seconds
world.resample_count = 20 -- partile filter cycles

-- Associate with the table
Config.world = world

return Config
