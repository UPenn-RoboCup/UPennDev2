module(..., package.seeall);
require('vector')

--Localization parameters for Testing in Grasp
--The field is shrinked to 85% of its real size
--But the size of the boxes and the distance between the goal posts are unchanged

world={};
world.n = 200;
world.xLineBoundary = 3;
world.yLineBoundary = 2;
--world.xMax = 4;
world.xMax = 3.2;
world.yMax = 2.2;
world.goalWidth = 1.60;
world.goalHeight= 0.85;
world.goalDiameter=0.10; -- diameter of a post
world.ballYellow= {{4.5,0.0}};
world.ballCyan= {{-4.6,0.0}};
world.postYellow = {};
world.postYellow[1] = {3, 0.80};
world.postYellow[2] = {3, -0.80};
world.postCyan = {};
world.postCyan[1] = {-3, -0.80};
world.postCyan[2] = {-3, 0.80};
world.spot = {};
world.spot[1] = {-1.2, 0};
world.spot[2] = {1.2, 0};

--They are SPL 2013 values
world.Lcorner={};
--Field edge
world.Lcorner[1]={3,2};
world.Lcorner[2]={3,-2};
world.Lcorner[3]={-3,2};
world.Lcorner[4]={-3,-2};
--Center T edge
world.Lcorner[5]={0,2};
world.Lcorner[6]={0,-2};
--Penalty box edge
world.Lcorner[7]={-2.4,1.1};
world.Lcorner[8]={-2.4,-1.1};
world.Lcorner[9]={2.4,1.1};
world.Lcorner[10]={2.4,-1.1};
--Penalty box T edge
world.Lcorner[11]={3,1.1};
world.Lcorner[12]={3,-1.1};
world.Lcorner[13]={-3,1.1};
world.Lcorner[14]={-3,-1.1};
--Center circle junction
world.Lcorner[15]={0,0.6};
world.Lcorner[16]={0,-0.6};
world.Lcorner[17]={0.6,0};
world.Lcorner[18]={-0.6,0};

--Goalie only uses corners near goals

world.Lgoalie_corner = {}
--Field edge
world.Lgoalie_corner[1]=world.Lcorner[1];
world.Lgoalie_corner[2]=world.Lcorner[2];
world.Lgoalie_corner[3]=world.Lcorner[3];
world.Lgoalie_corner[4]=world.Lcorner[4];

--Penalty box edge
world.Lgoalie_corner[5]=world.Lcorner[7];
world.Lgoalie_corner[6]=world.Lcorner[8];
world.Lgoalie_corner[7]=world.Lcorner[9];
world.Lgoalie_corner[8]=world.Lcorner[10];

--Penalty box T edge
world.Lgoalie_corner[9]=world.Lcorner[11];
world.Lgoalie_corner[10]=world.Lcorner[12];
world.Lgoalie_corner[11]=world.Lcorner[13];
world.Lgoalie_corner[12]=world.Lcorner[14];


--SJ: OP does not use yaw odometry data (only use gyro)
world.odomScale = {1, 1, 0};  
world.imuYaw = 1;
--Vision only testing (turn off yaw gyro)
--world.odomScale = {1, 1, 1};  
--world.imuYaw = 0;

-- default positions for our kickoff
world.initPosition1={
  {2.8,0},   --Goalie
  {0.5, 0}, --Attacker
  {1.5,-1.25}, --Defender
  {0.5, 1}, --Supporter
}
-- default positions for opponents' kickoff
-- Penalty mark : {1.2,0}
world.initPosition2={
  {2.8,0},   --Goalie
  {0.8, 0}, --Attacker
  {1.5, -0.5}, --Defender
  {1.75,1}, --Supporter
}

-- default positions for dropball
-- Center circle radius: 0.6
world.initPosition3={
  {2.0,0},   --Goalie
  {0.5,0}, --Attacker
  {1.5,-1.5}, --Defender
  {0.5,1.0}, --Supporter
}



--Resampling parameters
world.cResample = 10; --Resampling interval
world.daNoise = 2.0*math.pi/180;
world.drNoise = 0.01;

-- filter weights

--Sigma value for one landmark observation
world.rSigmaSingle1 = .15;
world.rSigmaSingle2 = 0.10;
--world.aSigmaSingle = 50*math.pi/180;
world.aSigmaSingle = 5*math.pi/180;

--Sigma value for two landmark observation
world.rSigmaDouble1 = .25;
world.rSigmaDouble2 = .20;
--world.aSigmaDouble = 50*math.pi/180;
world.aSigmaDouble = 5*math.pi/180;

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
--world.rCornerFilter = 0.01;
--world.aCornerFilter = 0.03;

world.rCornerFilter = 0.1;
world.aCornerFilter = 0.1;
--For line
--world.aLineFilter = 0.02;
world.aLineFilter = 0.075;


world.use_same_colored_goal = 1;
world.use_new_goalposts=1;
world.use_line_angles = 1;

world.triangulation_threshold = 4.0; 
world.position_update_threshold = 6.0;
world.angle_update_threshold = 1.0;
world.flip_correction = 0;

world.dont_reset_orientation = 1;

