module(..., package.seeall);
require('vector')

--Localization parameters 

world={};
world.n = 100;
world.xLineBoundary = 3.0;
world.yLineBoundary = 2.0;
world.xMax = 3.2;
world.yMax = 2.2;
world.goalWidth = 1.60;
world.goalHeight= 0.85;
world.ballYellow= {{3.0,0.0}};
world.ballCyan= {{-3.0,0.0}};
world.postYellow = {};
world.postYellow[1] = {3.0, 0.80};
world.postYellow[2] = {3.0, -0.80};
world.postCyan = {};
world.postCyan[1] = {-3.0, -0.80};
world.postCyan[2] = {-3.0, 0.80};
world.spot = {};
world.spot[1] = {-1.20, 0};
world.spot[2] = {1.20, 0};
world.landmarkCyan = {0.0, -2.4};
world.landmarkYellow = {0.0, 2.4};
world.cResample = 10; --Resampling interval

--SJ: OP does not use yaw odometry data (only use gyro)
world.odomScale = {1, 1, 0};  
world.imuYaw = 1;
--Vision only testing (turn off yaw gyro)
--world.odomScale = {1, 1, 1};  
--world.imuYaw = 0;

-- default positions for our kickoff
world.initPosition1={
  {3,0},   --Goalie
  {0.5,0}, --Attacker
  {1.5,-0.5}, --Defender
  {0.5,1.0}, --Supporter
}
-- default positions for opponents' kickoff
-- Center circle radius: 0.6
world.initPosition2={
  {3,0},   --Goalie
  {0.8,0}, --Attacker
  {1.5,0.5}, --Defender
  {1.75,-1.0}, --Supporter
}

-- filter weights
world.rGoalFilter = 0.02;
world.aGoalFilter = 0.05;
world.rPostFilter = 0.02;
world.aPostFilter = 0.20;

--New two-goalpost localization
world.use_new_goalposts=1;

-- Occupancy Map parameters
occmap = {};
occmap.div = 72;
