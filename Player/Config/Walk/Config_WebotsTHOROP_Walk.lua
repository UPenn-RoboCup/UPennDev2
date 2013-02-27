module(..., package.seeall); require('vector')
require 'unix'

----------------------------------------------------
-- Kneeling parameters
----------------------------------------------------
kneel = {};
kneel.bodyHeight = 0.45;
kneel.bodyTilt = 90*math.pi/180;

kneel.armX = 0.35;
kneel.armY = 0.296;
kneel.armZ = 0.078;
kneel.LArmRPY = {-math.pi/2,0,0};
kneel.RArmRPY = {math.pi/2,0,0};
kneel.legX = -0.395;
kneel.legY = 0.210;

kneel.tStep = 0.50;
kneel.stepHeight = 0.15;

kneel.velLimitX = {-.10,.20};
kneel.velLimitY = {-.5,.5};
kneel.velLimitA = {-.2,.2};

kneel.ph1Single = 0.2;
kneel.ph2Single = 0.8;
kneel.tZmp = 0.22;

--For wrist-walking 
kneel.armX = 0.26;
kneel.armZ = 0.02;

--High step testing
kneel.stepHeight = 0.20;
kneel.armX = 0.21;

kneel.torsoX = -(kneel.armX + kneel.legX)/2;

--only used for kneel down 
kneel.qLArm0 = {0.43,0.26,0.09,-1.15,-1.57,-1.57};


--[[
--Higher step testing
kneel.tStep = 1;
kneel.ph1Single = 0.1;
kneel.ph2Single = 0.9;
--]]





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

--gyroFactor = gyroFactor*3;
gyroFactorX = gyroFactor*1;
gyroFactorY = gyroFactor*1;


walk.ankleImuParamX={0.3,0.75*gyroFactorX, 0*math.pi/180, 5*math.pi/180};
walk.kneeImuParamX={0.3,1.5*gyroFactorX, 0*math.pi/180, 5*math.pi/180};

walk.ankleImuParamY={0.3,0.25*gyroFactorY, 0*math.pi/180, 2*math.pi/180};
walk.hipImuParamY={0.3,0.25*gyroFactorY, 0*math.pi/180, 2*math.pi/180};
walk.armImuParamX={1,10*gyroFactorX, 20*math.pi/180, 45*math.pi/180};
walk.armImuParamY={1,10*gyroFactorY, 20*math.pi/180, 45*math.pi/180};

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

--webots thor-op values
walk.bodyHeight = 1.15; 
walk.footX= 0.00;  --depreciated now
--Relative position of torso from COM position
--torsoX is -footX 
walk.torsoX = 0.00; 

walk.footY = 0.07;
walk.supportX = 0.0;
walk.supportY = 0.04;
walk.bodyTilt = 0*math.pi/180;

--walk.tZmp = 0.26; --Com height 0.65
--walk.tZmp = 0.34; --Com height 1.15
--walk.tZmp = 0.32; --Com height 1.0
walk.tZmp = 0.30; --Com height 0.9


walk.tStep = 0.8;
walk.stepHeight = 0.052;
walk.phSingle = {0.15,0.85};
walk.phZmp = {0.15,0.85};
walk.hipRollCompensation = 3*math.pi/180;

--Large stride test (up to 300mm)
walk.velLimitX={-.30,.30};
walk.stanceLimitX={-0.80,0.80};
walk.velDelta={0.30,0.10,0.3} 
walk.velXHigh = 0.30;

walk.supportModYInitial=-0.04; --Reduce initial body swing



-------------------------------------
-- ZMP preview stepping test
-- This works with 750mm stride

walk.tZmp = 0.28; --Com height 0.9
walk.supportY = 0.0;

--Larger stride test with ZMP preview
walk.velLimitX={-.60,.60};
walk.stanceLimitX={-.60,.60};
walk.velDelta={0.20,0.10,0.3} 
walk.velXHigh = 0.60;
walk.stepHeight = 0.10;
walk.footY = 0.10;
walk.supportX = 0.04;
walk.bodyHeight = 0.98; 
walk.supportX = 0.02;

-----------------------------------------
