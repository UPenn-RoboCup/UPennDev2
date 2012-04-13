module(..., package.seeall); require('vector')
require 'unix'
-- Walk Parameters

walk = {};

----------------------------------------------
-- Stance and velocity limit values
----------------------------------------------
walk.stanceLimitX={-0.10,0.10};
walk.stanceLimitY={0.07,0.20};
walk.stanceLimitA={0*math.pi/180,30*math.pi/180};
walk.velLimitX={-.04,.08};
walk.velLimitY={-.04,.04};
walk.velLimitA={-.4,.4};
walk.velDelta={0.02,0.02,0.15} 

walk.velLimitX={-.04,.09};

----------------------------------------------
-- Stance parameters
---------------------------------------------
walk.bodyHeight = 0.295; 
walk.bodyTilt=20*math.pi/180; 
walk.footX= -0.020; 
walk.footY = 0.035;
walk.supportX = 0;
walk.supportY = 0.010;
walk.qLArm=math.pi/180*vector.new({90,8,-40});
walk.qRArm=math.pi/180*vector.new({90,-8,-40});
walk.qLArmKick=math.pi/180*vector.new({90,30,-60});
walk.qRArmKick=math.pi/180*vector.new({90,-30,-60});

walk.hardnessSupport = 1;
walk.hardnessSwing = 1;
walk.hardnessArm=.3;
---------------------------------------------
-- Gait parameters
---------------------------------------------
walk.tStep = 0.25;
walk.tZmp = 0.165;
walk.stepHeight = 0.035;
walk.phSingle={0.1,0.9};

--------------------------------------------
-- Compensation parameters
--------------------------------------------
walk.hipRollCompensation = 4*math.pi/180;
walk.ankleMod = vector.new({-1,0})/0.12 * 10*math.pi/180;

--------------------------------------------------------------
--Imu feedback parameters, alpha / gain / deadband / max
--------------------------------------------------------------
gyroFactor = 0.273*math.pi/180 * 300 / 1024; --dps to rad/s conversion

walk.ankleImuParamX={0.9,0.3*gyroFactor, 0, 25*math.pi/180};
walk.kneeImuParamX={0.9,1.2*gyroFactor, 0, 25*math.pi/180};
walk.ankleImuParamY={0.9,0.7*gyroFactor, 0, 25*math.pi/180};
walk.hipImuParamY={0.9,0.3*gyroFactor, 0, 25*math.pi/180};
walk.armImuParamX={0.3,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};
walk.armImuParamY={0.3,10*gyroFactor, 20*math.pi/180, 45*math.pi/180};

--------------------------------------------
-- Support point modulation values
--------------------------------------------
walk.supportFront = 0.01; --Lean front when walking fast forward
walk.supportBack = -0.02; --Lean back when walking backward
walk.supportSideX = -0.01; --Lean back when sidestepping
walk.supportSideY = 0.02; --Lean sideways when sidestepping

--------------------------------------------
-- WalkKick parameters
--------------------------------------------
walk.walkKickDef={}

--tStep stepType supportLeg stepHeight 
-- SupportMod shiftFactor footPos1 footPos2

walk.walkKickDef["FrontLeft"]={
  {0.30, 1, 0, 0.035 , {0,0}, 0.7, {0.06,0,0} },
  {0.30, 2, 1, 0.07 , {0.02,-0.02}, 0.5, {0.09,0,0}, {0.06,0,0} },
  {walk.tStep, 1, 0, 0.035 , {0,0}, 0.5, {0,0,0} },
}
walk.walkKickDef["FrontRight"]={
  {0.30, 1, 1, 0.035 , {0,0}, 0.3, {0.06,0,0} },
  {0.30, 2, 0, 0.07 , {0.02,0.02}, 0.5,  {0.09,0,0}, {0.06,0,0} },
  {walk.tStep, 1, 1, 0.035 , {0,0}, 0.5, {0,0,0} },
}
walk.walkKickDef["SideLeft"]={
  {0.30, 1, 1, 0.035 , {0,0}, 0.3, {0.04,0.04,0} },
  {0.35, 3, 0, 0.07 , {-0.01,0.01}, 0.5, {0.06,-0.05,0},{0.09,0.01,0}},
 {0.25, 1, 1, 0.035 , {0,0}, 0.5, {0,0,0} },}

walk.walkKickDef["SideRight"]={
  {0.30, 1, 0, 0.035 , {0,0}, 0.7, {0.04,-0.04,0} },
  {0.35, 3, 1, 0.07 , {-0.01,-0.01},0.5, {0.06,0.05,0},{0.09,-0.01,0}},
  {0.25, 1, 0, 0.035 , {0,0},0.5,  {0,0,0} },
}

walk.walkKickPh=0.5;





--Fall detection angle... OP requires large angle
walk.fallAngle = 50*math.pi/180;

--------------------------------------------
-- Robot - specific calibration parameters
--------------------------------------------

walk.kickXComp = 0;
walk.supportCompL = {0,0,0};
walk.supportCompR = {0,0,0};
walk.servoBias = {0,0,0,0,0,0,0,0,0,0,0,0};
walk.footXComp = 0;
walk.footYComp = 0;

--Default pitch angle offset of OP 
walk.headPitchBias = 40* math.pi / 180; 
walk.headPitchBiasComp = 0;

local robotName = unix.gethostname();
local robotID = 0;

--Load robot specific calibration value
require('calibration');
if calibration.cal and calibration.cal[robotName] then
  walk.servoBias = calibration.cal[robotName].servoBias;
  walk.footXComp = calibration.cal[robotName].footXComp;
  walk.kickXComp = calibration.cal[robotName].kickXComp;
  walk.headPitchBiasComp = calibration.cal[robotName].headPitchBiasComp;
  print(robotName.." walk parameters loaded")
end





-- Slow walk
--[[
walk.tZmp = 0.165;
walk.tStep = 0.5;
walk.phSingle={0.1,0.9};
walk.supportY = 0.010;
walk.supportX = -0.005;
walk.stepHeight = 0.06;
walk.qLArm=math.pi/180*vector.new({90,0,-80});
walk.qRArm=math.pi/180*vector.new({90,0,-80});
--]]


-- Grip walk
--[[
--walk.bodyHeight = 0.285;
--walk.bodyTilt=0*math.pi/180;
walk.tZmp = 0.165;
walk.tStep = 0.5;
walk.phSingle={0.25,0.75};
walk.footX = -0.010;
walk.footY = 0.050;
walk.supportY = 0.000;
walk.supportX = 0.000;
walk.stepHeight = 0.04;
walk.qLArm=math.pi/180*vector.new({26.2, 90, 33.4});
walk.qRArm=math.pi/180*vector.new({0,50,120});
--]]


