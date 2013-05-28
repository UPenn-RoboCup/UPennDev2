module(..., package.seeall);
require('vector')

--Localization parameters 

world={};
world.n = 200;
world.xLineBoundary = 4.5;
world.yLineBoundary = 3.0;
world.xMax = 4.7;
world.yMax = 3.2;
world.goalWidth = 1.60;
world.goalHeight= 0.85;
world.goalDiameter=0.10; -- diameter of a post
world.ballYellow= {{4.5,0.0}};
world.ballCyan= {{-4.6,0.0}};
world.postYellow = {};
world.postYellow[1] = {4.5, 0.80};
world.postYellow[2] = {4.5, -0.80};
world.postCyan = {};
world.postCyan[1] = {-4.5, -0.80};
world.postCyan[2] = {-4.5, 0.80};
world.spot = {};
world.spot[1] = {-2.70, 0};
world.spot[2] = {2.70, 0};
world.cResample = 10; --Resampling interval

--They are SPL values
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
world.Lcorner[7]={-3.9,1.1};
world.Lcorner[8]={-3.9,-1.1};
world.Lcorner[9]={3.9,1.1};
world.Lcorner[10]={3.9,-1.1};
--Penalty box T edge
world.Lcorner[11]={4.5,1.1};
world.Lcorner[12]={4.5,-1.1};
world.Lcorner[13]={-4.5,1.1};
world.Lcorner[14]={-4.5,-1.1};
--Center circle junction
world.Lcorner[15]={0,0.6};
world.Lcorner[16]={0,-0.6};
world.Lcorner[17]={0.6,0};
world.Lcorner[18]={-0.6,0};

--same-colored goalposts
world.use_same_colored_goal=1;

--should we use new triangulation?
world.use_new_goalposts=0;

-- filter weights
world.rGoalFilter = 0.02;
world.aGoalFilter = 0.05;
world.rPostFilter = 0.02;
world.aPostFilter = 0.05;
world.rKnownGoalFilter = 0.02;
world.aKnownGoalFilter = 0.20;
world.rKnownPostFilter = 0.02;
world.aKnownPostFilter = 0.10;
world.rUnknownGoalFilter = 0.02;
world.aUnknownGoalFilter = 0.05;
world.rUnknownPostFilter = 0.02;
world.aUnKnownPostFilter = 0.05;

world.rCornerFilter = 0.02;
world.aCornerFilter = 0.05;

-- default positions for our kickoff
world.initPosition1={
  {4.2,0},   --Goalie
  {0.5, 0}, --Attacker
  {1.2,-1}, --Defender
  {1.2, 1}, --Supporter
  {2.2, 0}, --Defender2
}
-- default positions for opponents' kickoff
-- Penalty mark : {1.2,0}
world.initPosition2={
  {4.2,0},   --Goalie
  {2.0, 0}, --Attacker
  {2.5, -1}, --Defender
  {2.5,1}, --Supporter
  {2.5, 0}, --Defender2
}

-- use sound localization
world.enable_sound_localization = 0;


