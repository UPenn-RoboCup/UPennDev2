module(..., package.seeall);
require('vector')

--Localization parameters 

world={};
world.n = 100;
world.xLineBoundary = 3.0;
world.yLineBoundary = 2.0;
world.xMax = 3.2;
world.yMax = 2.2;
world.goalWidth = 1.40;
world.goalHeight= 0.80;
world.ballYellow= {{3.0,0.0}};
world.ballCyan= {{-3.0,0.0}};
world.postYellow = {};
world.postYellow[1] = {3.0, 0.70};
world.postYellow[2] = {3.0, -0.70};
world.postCyan = {};
world.postCyan[1] = {-3.0, -0.70};
world.postCyan[2] = {-3.0, 0.70};
world.spot = {};
world.spot[1] = {-1.20, 0};
world.spot[2] = {1.20, 0};
world.landmarkCyan = {0.0, -2.4};
world.landmarkYellow = {0.0, 2.4};
world.cResample = 10; --Resampling interval
world.odomScale = {1.06, 1.06, 0.97};

--For same-colored goalposts
world.use_same_colored_goal=1;
world.postUnified={};
world.postUnified[1] = {3.0, 0.70};
world.postUnified[2] = {3.0, -0.70};
world.postUnified[3] = {-3.0, -0.70};
world.postUnified[4] = {-3.0, 0.70};
world.postLeft={};
world.postLeft[1] = {3.0, 0.70};
world.postLeft[2] = {-3.0, -0.70};
world.postRight={};
world.postRight[1] = {3.0, -0.70};
world.postRight[2] = {-3.0, 0.70};

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
