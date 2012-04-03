module(..., package.seeall); require('vector')
require 'unix'
-- Walk Parameters for NewWalk

walk = {};

----------------------------------------------
-- Stance and velocity limit values
----------------------------------------------
walk.stanceLimitX={-0.10,0.10};
walk.stanceLimitY={0.09,0.20};
walk.stanceLimitA={0*math.pi/180,40*math.pi/180};
walk.velLimitX={-.06,.08};
walk.velLimitY={-.06,.06};
walk.velLimitA={-.4,.4};
walk.velDelta={0.03,0.015,0.15} 

----------------------------------------------
-- Stance parameters
---------------------------------------------
walk.bodyHeight = 0.30; 
walk.bodyTilt=0*math.pi/180; 
walk.footX= 0.0; 
walk.footY = 0.0475;
walk.supportX = 0.016;
walk.supportY = 0.025;
walk.qLArm = math.pi/180*vector.new({105, 12, -85, -30});
walk.qRArm = math.pi/180*vector.new({105, -12, 85, 30});

walk.hardnessSupport = .7;
walk.hardnessSwing = .5;
walk.hardnessArm=.3;

---------------------------------------------
-- Gait parameters
---------------------------------------------
walk.tStep = 0.44;
walk.tZmp = 0.17;
walk.stepHeight = 0.020;
walk.phSingle={0.16,0.84};

--------------------------------------------
-- Compensation parameters
--------------------------------------------
walk.hipRollCompensation = 0*math.pi/180;
walk.ankleMod = vector.new({-1,0})/0.12 * 10*math.pi/180;





--Webots FIX
walk.tStep = 0.48;
walk.supportX = 0.010;
walk.supportY = 0.035;
walk.phSingle={0.2,0.8};
walk.velLimitY={-.05,.05};
walk.hipRollCompensation = 2*math.pi/180;

--------------------------------------------------------------
--Imu feedback parameters, alpha / gain / deadband / max
--------------------------------------------------------------
gyroFactor = 0.273*math.pi/180 * 300 / 1024; --dps to rad/s conversion
gyroFactor = 0;
walk.ankleImuParamX={1,-0.75*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.kneeImuParamX={1,-1.5*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.ankleImuParamY={1,-1*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.hipImuParamY={1,-1*gyroFactor, 2*math.pi/180, 10*math.pi/180};
walk.armImuParamX={0.3,-10*gyroFactor, 20*math.pi/180, 45*math.pi/180};
walk.armImuParamY={0.3,-10*gyroFactor, 20*math.pi/180, 45*math.pi/180};

--------------------------------------------
-- WalkKick parameters
--------------------------------------------
walk.walkKickVel = {0.06, 0.14} --step / kick / follow 
walk.walkKickSupportMod = {{-0.03,0},{-0.03,0}}
walk.walkKickHeightFactor = 3.0;

walk.qLArmKick = math.pi/180*vector.new({105, 12, -85, -30});
walk.qRArmKick = math.pi/180*vector.new({105, -12, 85, 30});

walk.sideKickVel1 = {0.04,0.04};
walk.sideKickVel2 = {0.09,0.05};
walk.sideKickVel3 = {0.09,-0.02};
walk.sideKickSupportMod = {{0,0},{0,0}};
walk.tStepSideKick = 0.70;

--------------------------------------------
-- Robot - specific calibration parameters
--------------------------------------------

walk.kickXComp = 0;
walk.supportCompL = {0,0,0};
walk.supportCompR = {0,0,0};

