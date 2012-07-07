module(..., package.seeall);
require('vector')

--Localization parameters 

world={};
world.n = 100;
world.xLineBoundary = 4.5;
world.yLineBoundary = 3.0;
world.xMax = 4.7;
world.yMax = 3.2;
world.goalWidth = 2.70;
world.goalHeight= 1.85;
world.ballYellow= {{3.0,0.0}};
world.ballCyan= {{-3.0,0.0}};
world.postYellow = {};
world.postYellow[1] = {4.5, 1.35};
world.postYellow[2] = {4.5, -1.35};
world.postCyan = {};
world.postCyan[1] = {-4.5, -1.35};
world.postCyan[2] = {-4.5, 1.35};
world.spot = {};
world.spot[1] = {-2.40, 0};
world.spot[2] = {2.40, 0};
world.landmarkCyan = {0.0, -3.4};
world.landmarkYellow = {0.0, 3.4};
world.cResample = 10; --Resampling interval


--Actual Teen/Adultsize values
world.Lcorner={};
--Field edge
world.Lcorner[1]={4.5,3.0};
world.Lcorner[2]={4.5,-3.0};
world.Lcorner[3]={-4.5,3.0};
world.Lcorner[4]={-4.5,-3.0};
--Center T edge
world.Lcorner[5]={0,3.0};
world.Lcorner[6]={0,-3.0};
--Penalty box edge
world.Lcorner[7]={-3.5,2.25};
world.Lcorner[8]={-3.5,-2.25};
world.Lcorner[9]={3.5,2.25};
world.Lcorner[10]={3.5,-2.25};
--Penalty box T edge
world.Lcorner[11]={4.5,2.25};
world.Lcorner[12]={4.5,-2.25};
world.Lcorner[13]={-4.5,2.25};
world.Lcorner[14]={-4.5,-2.25};
--Center circle junction
world.Lcorner[15]={0,0.75};
world.Lcorner[16]={0,-0.75};

--Temporary Webot field values (SPL * 1.5)
--Penalty box edge
world.Lcorner[7]={-3.6,1.65};
world.Lcorner[8]={-3.6,-1.65};
world.Lcorner[9]={3.6,1.65};
world.Lcorner[10]={3.6,-1.65};
--Penalty box T edge
world.Lcorner[11]={4.5,1.65};
world.Lcorner[12]={4.5,-1.65};
world.Lcorner[13]={-4.5,1.65};
world.Lcorner[14]={-4.5,-1.65};
--Center circle junction
world.Lcorner[15]={0,0.9};
world.Lcorner[16]={0,-0.9};



--SJ: OP does not use yaw odometry data (only use gyro)
world.odomScale = {1, 1, 0};  
world.imuYaw = 1;

--For Adult and Teensize field
world.initPosition1={
  {4.5,0},   --Goalie
  {0,0}, --Attacker
  {2,0}, --Defender
  {2,2}, --Supporter
}
-- default positions for opponents' kickoff
-- Penalty mark : {1.2,0}
world.initPosition2={
  {4.5,0},   --Goalie
  {0.8,0}, --Attacker
  {2,0}, --Defender
  {2,2}, --Supporter
}


-- filter weights
world.rGoalFilter = 0.02;
world.aGoalFilter = 0.05;
world.rPostFilter = 0.02;
world.aPostFilter = 0.20;

-- Occupancy Map parameters
occmap = {};
occmap.div = 72;

-- Occupancy Map parameters
occ = {};
occ.mapsize = 50;
occ.centroid = {occ.mapsize / 2, occ.mapsize * 4 / 5};


