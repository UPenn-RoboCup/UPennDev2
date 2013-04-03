module(..., package.seeall); require('vector')
require 'unix'
-- Walk Parameters

walk = {};

----------------------------------------------
-- Stance and velocity limit values
----------------------------------------------
walk.stanceLimitX={-0.60,0.60};
walk.stanceLimitY={0.16,0.60};
walk.stanceLimitA={-10*math.pi/180,30*math.pi/180};
walk.velLimitX={-.20,.20};
walk.velLimitY={-.10,.10};
walk.velLimitA={-.2,.2};
walk.velDelta={0.10,0.10,0.3} 

----------------------------------------------
-- Stance parameters
---------------------------------------------
walk.bodyHeight = 0.75; 
walk.bodyTilt=4*math.pi/180; 
walk.footX= 0.01; 
walk.footY = 0.09;
walk.supportX = 0;
walk.supportY = 0.0;
walk.qLArm = math.pi/180*vector.new({110, 12, -0, -40});
walk.qRArm = math.pi/180*vector.new({110, -12, 0, 40});
walk.qLArmKick = math.pi/180*vector.new({110, 12, -0, -40});
walk.qRArmKick = math.pi/180*vector.new({110, -12, 0, 40});

walk.hardnessSupport = 1;
walk.hardnessSwing = 1;
walk.hardnessArm=.1;
walk.hardnessArm=0;
---------------------------------------------
-- Gait parameters
---------------------------------------------
walk.tStep = 1.0;
walk.tZmp = 0.25;
walk.stepHeight = 0.06;
walk.phSingle={0.1,0.9};
walk.phZmp={0.1,0.9};

--------------------------------------------
-- Compensation parameters
--------------------------------------------
walk.hipRollCompensation = 0*math.pi/180;
walk.ankleMod = vector.new({0,0})/ 3*math.pi/180;

--------------------------------------------
-- Support point modulation values
--------------------------------------------
walk.supportFront = 0.01; --Lean front when walking fast forward
walk.supportBack = -0.02; --Lean back when walking backward
walk.supportSideX = -0.01; --Lean back when sidestepping
walk.supportSideY = 0.02; --Lean sideways when sidestepping

-------------------------------------------------------------- 
--Imu feedback parameters, alpha / gain / deadband / max 
-------------------------------------------------------------- 
gyroFactor = 0.273* 300 / 1024; --dps to rad/s conversion

walk.ankleImuParamX={1,0.75*gyroFactor, 2*math.pi/180, 5*math.pi/180};
walk.kneeImuParamX={1,1.5*gyroFactor, 2*math.pi/180, 5*math.pi/180};
walk.ankleImuParamY={1,0.75*gyroFactor, 2*math.pi/180, 5*math.pi/180};
walk.hipImuParamY={1,0.25*gyroFactor, 2*math.pi/180, 5*math.pi/180};

walk.armImuParamX={0,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};
walk.armImuParamY={0,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};

--------------------------------------------
-- WalkKick parameters
--------------------------------------------

walk.walkKickDef={}

--tStep stepType supportLeg stepHeight SupportMod shiftFactor footPos1 footPos2

walk.walkKickDef["FrontLeft"]={
  {0.75, 1, 0, 0.052 , {0,0}, 0.7, {0.15,0,0} },
  {1.5, 2, 1, 0.060 , {0.02,-0.02}, 0.7, {0.15,0,0}, {0.10,0,0} },
  {walk.tStep, 1, 0, 0.035 , {0,0}, 0.5, {0,0,0} },
}
walk.walkKickDef["FrontRight"]={
  {0.75, 1, 1, 0.052 , {0,0}, 0.3, {0.15,0,0} },
  {1.5, 2, 0, 0.060, {0.02,0.02}, 0.5,  {0.15,0,0}, {0.10,0,0} },
  {walk.tStep, 1, 1, 0.035 , {0,0}, 0.5, {0,0,0} },
}
walk.walkKickDef["SideLeft"]={
  {0.75, 1, 1, 0.035 , {0,0}, 0.3, {0.08,0.10,0} },
  {0.75, 3, 0, 0.07 , {-0.01,0.06}, 0.5, {0.18,-0.20,0},{0.18,0.0,0}},
 {walk.tStep, 1, 1, 0.035 , {0,0}, 0.5, {0,0,0} },}

walk.walkKickDef["SideRight"]={
  {0.75, 1, 0, 0.035 , {0,0}, 0.7, {0.08,-0.10,0} },
  {0.75, 3, 1, 0.07 , {-0.01,-0.06},0.5, {0.18,0.20,0},{0.18,-0.0,0}},
  {walk.tStep, 1, 0, 0.035 , {0,0},0.5,  {0,0,0} },
}

walk.walkKickPh=0.5;

--------------------------------------------
-- Robot - specific calibration parameters
--------------------------------------------

walk.kickXComp = 0;
walk.supportCompL = {0,0,0};
walk.supportCompR = {0,0,0};

walk.kickXComp = 0;
walk.supportCompL = {0,0,0};
walk.supportCompR = {0,0,0};
walk.servoBias = {0,0,0,0,0,0,0,0,0,0,0,0};
walk.footXComp = 0;
walk.footYComp = 0;

--Default pitch angle offset of Charli 
walk.headPitchBias = 0* math.pi / 180; 

--Slow and stable walk (like 2011)

walk.bodyTilt=4*math.pi/180; 

walk.stepHeight = 0.052;
walk.footX = -0.03;
walk.footY = 0.10;
walk.supportX = 0.0;
walk.supportY = 0.02;

walk.tStep = 0.75;
walk.tZmp = 0.26; --Com height 0.65
walk.tZmp = 0.265; 

walk.hipRollCompensation = 5*math.pi/180;
walk.phSingle = {0.15,0.85};
walk.phZmp = {0.15,0.85};
walk.supportModYInitial=-0.04; --Reduce initial body swing



walk.supportX = -0.02;
walk.supportY = 0.03;
walk.supportModYInitial=-0.06; --Reduce initial body swing

--Lean sideways when sidestepping
walk.supportSideY = 0.01; 

walk.supportY = 0.028;
walk.tZmp = 0.27; 


