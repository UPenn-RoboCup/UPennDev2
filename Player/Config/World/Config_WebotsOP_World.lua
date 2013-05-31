module(..., package.seeall);
require('vector')

--Localization parameters 

world={};
--world.n = 100;
world.n = 200;
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
world.cResample = 10; --Resampling interval

--Field edge
--SJ: rule change in 2013 (penalty box width 2.2m)

world.Lcorner={};
world.Lcorner[1]={3.0,2.0};
world.Lcorner[2]={3.0,-2.0};
world.Lcorner[3]={-3.0,2.0};
world.Lcorner[4]={-3.0,-2.0};

--Center T edge
world.Lcorner[5]={0,2.0};
world.Lcorner[6]={0,-2.0};

--Penalty box edge
world.Lcorner[7]={-2.4,1.1};
world.Lcorner[8]={-2.4,-1.1};
world.Lcorner[9]={2.4,1.1};
world.Lcorner[10]={2.4,-1.1};

--Penalty box T edge
world.Lcorner[11]={3.0,1.1};
world.Lcorner[12]={3.0,-1.1};
world.Lcorner[13]={-3.0,1.1};
world.Lcorner[14]={-3.0,-1.1};

--Center circle junction
world.Lcorner[15]={0,0.6};
world.Lcorner[16]={0,-0.6};




--SJ: OP does not use yaw odometry data (only use gyro)
world.odomScale = {1, 1, 0};  
world.gyroScale = 1; --heuristic value to prevent overshooting


world.imuYaw = 1;
--Vision only testing (turn off yaw gyro)
--world.odomScale = {1, 1, 1};  
--world.imuYaw = 0;

-- default positions for our kickoff
world.initPosition1={
  {2.8,0},   --Goalie
  {0.5,0}, --Attacker
  {1.5,-1.25}, --Defender
  {0.5,1.0}, --Supporter
}
-- default positions for opponents' kickoff
-- Center circle radius: 0.6
world.initPosition2={
  {2.8,0},   --Goalie
  {0.8,0}, --Attacker
  {1.5,-0.5}, --Defender
  {1.75,1.0}, --Supporter
}

-- default positions for dropball
-- Center circle radius: 0.6
world.initPosition3={
  {2.8,0},   --Goalie
  {0.5,0}, --Attacker
  {1.5,-1.5}, --Defender
  {0.5,1.0}, --Supporter
}

-- filter weights
--old values
-------------------------------
world.rGoalFilter = 0.02;
world.aGoalFilter = 0.05;
world.rPostFilter = 0.02;
world.aPostFilter = 0.05;
world.rKnownGoalFilter = 0.02;
world.aKnownGoalFilter = 0.20;
world.rKnownPostFilter = 0.02;
world.aKnownPostFilter = 0.10;
----------------------------------

--for general update(corner, distant goalpost, etc)
world.rLandmarkFilter = 0.05;
world.aLandmarkFilter = 0.10;

--for Two goalposts
world.rUnknownGoalFilter = 0.02;
world.aUnknownGoalFilter = 0.05;

--For One goalpost
world.rUnknownPostFilter = 0.02;
world.aUnKnownPostFilter = 0.05;

--For corner
world.rCornerFilter = 0.01;
world.aCornerFilter = 0.03;

--For line
world.aLineFilter = 0.02;


world.use_same_colored_goal = 1;

--Use line information to fix angle
world.use_line_angles = 1;

--New two-goalpost localization
world.use_new_goalposts=1;
