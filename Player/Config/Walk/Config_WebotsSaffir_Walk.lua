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
walk.velLimitX={-.20,.40};
walk.velLimitY={-.15,.15};
walk.velLimitA={-.3,.3};
walk.velDelta={0.10,0.10,0.15} 

----------------------------------------------
-- Stance parameters
---------------------------------------------
walk.bodyHeight = 0.75; 
walk.bodyTilt=4*math.pi/180; 
walk.footX= 0.01; 
walk.footY = 0.10;
walk.supportX = 0;
walk.supportY = 0.0;
walk.qLArm = math.pi/180*vector.new({110, 12, -0, -90});
walk.qRArm = math.pi/180*vector.new({110, -12, 0, 90});
walk.hardnessSupport = 1;
walk.hardnessSwing = 1;
walk.hardnessArm=.1;
---------------------------------------------
-- Gait parameters
---------------------------------------------
walk.tStep = 1.0;
walk.tZmp = 0.25;
walk.stepHeight = 0.13;
walk.phSingle={0.1,0.9};
walk.phZmp={0.1,0.9};

--------------------------------------------
-- Compensation parameters
--------------------------------------------
walk.hipRollCompensation = 0*math.pi/180;
walk.ankleMod = vector.new({-1,0})/0.12 * 10*math.pi/180;

-------------------------------------------------------------- 
--Imu feedback parameters, alpha / gain / deadband / max 
-------------------------------------------------------------- 
gyroFactor = 0.273*math.pi/180 * 300 / 1024; --based on deg/s unit


gyroFactor = 0.273*math.pi/180 * 300 / 1024*0.2; --based on deg/s unit

walk.ankleImuParamX={1,0.75*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.kneeImuParamX={1,1.5*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.ankleImuParamY={1,1*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.hipImuParamY={1,1*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.armImuParamX={0.3,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};
walk.armImuParamY={0.3,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};

--------------------------------------------
-- WalkKick parameters
--------------------------------------------
walk.walkKickVel = {0.06, 0.14} --step / kick / follow 
walk.walkKickSupportMod = {{0,0},{0,0}}
walk.walkKickHeightFactor = 1.5;

--------------------------------------------
-- Robot - specific calibration parameters
--------------------------------------------

walk.kickXComp = 0;
walk.supportCompL = {0,0,0};
walk.supportCompR = {0,0,0};

--[[
walk.tStep = 0.8;
walk.tZmp = 0.25;
walk.supportY = -0.03;
walk.footY = 0.09;
walk.stepHeight = 0.13;
walk.hardnessSupport = .7;
walk.hardnessSwing = 0.2;
--]]




walk.velLimitA={-.6,.6};
walk.stanceLimitA={-10*math.pi/180,45*math.pi/180};
walk.hipRollCompensation = 1*math.pi/180;

--walk.phZmp={0.05,0.95};

--Fast walking test
gyroFactor = gyroFactor * 0.5;
walk.tStep = 0.7;
walk.velLimitY={-.10,.10};
walk.supportY = -0.01;
walk.tZmp = 0.20;
walk.supportX = -0.01;
walk.stepHeight = 0.08;
walk.phSingle={0.15,0.85};
walk.phZmp={0.15,0.85};

walk.velLimitX={-.20,.30};
walk.velDelta={0.10,0.10,0.3} 

--fast test
walk.tStep = 0.6;
walk.supportY = -0.01;
walk.footY = 0.09;
walk.velLimitA={-.2,.2};


--for SAFFIR
walk.stepHeight = 0.04;
walk.supportY = -0.01;
walk.tZmp = 0.25;
walk.velLimitA={-.3,.3};

walk.footX= 0.01; 
walk.supportX = -0.03;

walk.hipRollCompensation = 1.5*math.pi/180;

------------------------------
--For extended-knee walking
walk.footHeight = 0.0487;
walk.legLength = 0.379025+0.3800;
walk.hipOffsetX = 0.00;
walk.hipOffsetY = 0.097;
walk.hipOffsetZ = 0.0;
--walk.bodyTilt=0*math.pi/180; 
-------------------------------

--Running test
--------------------------------
walk.tStep = 0.4;
walk.bodyTilt=2*math.pi/180; 
walk.velLimitX={-.20,.30};
walk.footX= 0.01; 
walk.bodyTilt=2*math.pi/180; 
walk.phSingle={0.05,0.95};
walk.phZmp={0.05,0.95};
walk.bodyZ={0.03,0.02,-0.02,0.03};
walk.bodyK={2,2,2,2};
walk.phZ={0.15,0.5,0.85};
walk.torsoFactor=2.0;
--------------------------------

walk.tStep = 0.3;
walk.velLimitX={-.20,.30};


