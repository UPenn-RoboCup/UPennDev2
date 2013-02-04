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
walk.ankleMod = vector.new({-1,0})/ 3*math.pi/180;

-------------------------------------------------------------- 
--Imu feedback parameters, alpha / gain / deadband / max 
-------------------------------------------------------------- 
gyroFactor = 0.273*math.pi/180 * 300 / 1024; --dps to rad/s conversion

--gyroFactor = gyroFactor*0.3;


walk.ankleImuParamX={0.3,0.75*gyroFactor, 0*math.pi/180, 5*math.pi/180};
walk.kneeImuParamX={0.3,1.5*gyroFactor, 0*math.pi/180, 5*math.pi/180};
walk.ankleImuParamY={0.3,0.25*gyroFactor, 0*math.pi/180, 2*math.pi/180};
walk.hipImuParamY={0.3,0.25*gyroFactor, 0*math.pi/180, 2*math.pi/180};

walk.armImuParamX={1,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};
walk.armImuParamY={1,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};

--------------------------------------------
-- WalkKick parameters
--------------------------------------------

walk.walkKickDef={}

--tStep stepType supportLeg stepHeight SupportMod shiftFactor footPos1 footPos2

walk.walkKickDef["FrontLeft"]={
  {1.00, 1, 0, 0.035 , {0,0}, 0.7, {0.15,0,0} },
  {1.00, 2, 1, 0.07 , {0.02,-0.02}, 0.5, {0.25,0,0}, {0.15,0,0} },
  {walk.tStep, 1, 0, 0.035 , {0,0}, 0.5, {0,0,0} },
}
walk.walkKickDef["FrontRight"]={
  {1.00, 1, 1, 0.035 , {0,0}, 0.3, {0.15,0,0} },
  {1.00, 2, 0, 0.07 , {0.02,0.02}, 0.5,  {0.25,0,0}, {0.15,0,0} },
  {walk.tStep, 1, 1, 0.035 , {0,0}, 0.5, {0,0,0} },
}
walk.walkKickDef["SideLeft"]={
  {1.00, 1, 1, 0.035 , {0,0}, 0.3, {0.08,0.10,0} },
  {1.00, 3, 0, 0.07 , {-0.01,0.06}, 0.5, {0.18,-0.20,0},{0.18,0.04,0}},
 {walk.tStep, 1, 1, 0.035 , {0,0}, 0.5, {0,0,0} },}

walk.walkKickDef["SideRight"]={
  {1.00, 1, 0, 0.035 , {0,0}, 0.7, {0.08,-0.10,0} },
  {1.00, 3, 1, 0.07 , {-0.01,-0.06},0.5, {0.18,0.20,0},{0.18,-0.04,0}},
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

--[[
walk.tStep = 0.8;
walk.tZmp = 0.25;
walk.supportY = -0.03;
walk.footY = 0.09;
walk.stepHeight = 0.13;
walk.hardnessSupport = .7;
walk.hardnessSwing = 0.2;
--]]


--Slow and stable walk (like 2011)


walk.hipRollCompensation = 3*math.pi/180;

walk.tStep = 0.75;
walk.tZmp = 0.26; --Com height 0.65
--walk.tZmp = 0.20; --Com height 0.65
walk.supportX = 0.03;
walk.supportY = 0.02;
walk.bodyTilt = 0*math.pi/180;
walk.phSingle = {0.15,0.85};
walk.phZmp = {0.15,0.85};
walk.stepHeight = 0.052;

--[[
--Ridiculously fast walking (for webots)
walk.tStep = 0.35;
walk.supportY = 0.06;
--]]

walk.supportModYInitial=-0.04; --Reduce initial body swing
