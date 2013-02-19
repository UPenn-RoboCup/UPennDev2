module(..., package.seeall); require('vector')
require 'unix'

----------------------------------------------------
-- Kneeling parameters
----------------------------------------------------
kneel = {};
kneel.bodyHeight = 0.50;
kneel.bodyTilt = 90*math.pi/180;
--kneel.armX = 0.294;
kneel.armX = 0.35;
kneel.armY = 0.296;
kneel.armZ = 0.078;
kneel.LArmRPY = {-math.pi/2,0,0};
kneel.RArmRPY = {math.pi/2,0,0};
kneel.footX = -0.395;
kneel.footY = 0.210;
















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
walk.qLArm = math.pi/180*vector.new({110, 12, -0, -40,0,0});
walk.qRArm = math.pi/180*vector.new({110, -12, 0, -40,0,0});
walk.qLArmKick = math.pi/180*vector.new({110, 12, -0, -40,0,0});
walk.qRArmKick = math.pi/180*vector.new({110, -12, 0, -40,0,0});

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


--gyroFactor = gyroFactor*3;


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


walk.hipRollCompensation = 3*math.pi/180;

walk.tStep = 0.75;
walk.tZmp = 0.26; --Com height 0.65
walk.supportX = 0.03;
walk.supportY = 0.02;
walk.bodyTilt = 0*math.pi/180;
walk.phSingle = {0.15,0.85};
walk.phZmp = {0.15,0.85};
walk.stepHeight = 0.052;

walk.supportModYInitial=-0.04; --Reduce initial body swing

--webots thor-op values
walk.bodyHeight = 1.15; 
walk.footX= 0.005; 
walk.footY = 0.07;
-----------------------


--Very fast walk
walk.tStep = 0.45;

--Large stride test (up to 300mm)
walk.velLimitX={-.30,.30};
walk.stanceLimitX={-0.80,0.80};
walk.velDelta={0.30,0.10,0.3} 
walk.velXHigh = 0.30;
walk.supportY = 0.04;
walk.tStep = 0.8;



--Slow, High, Long stride walk (for stair climbing)
--[[
walk.phSingle = {0.2,0.8};
walk.phZmp = {0.2,0.8};
walk.hipRollCompensation = 3*math.pi/180;
walk.stepHeight = 0.25;
walk.tStep = 1;
--]]

--Wider stance
--walk.footY = 0.09;
