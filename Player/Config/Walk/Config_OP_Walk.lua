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
walk.qLArmKick=math.pi/180*vector.new({90,15,-40});
walk.qRArmKick=math.pi/180*vector.new({90,-15,-40});

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
walk.supportBack = -0.01; --Lean back when walking backward
walk.supportSide = 0.01; --Lean sideways when sidestepping

--------------------------------------------
-- WalkKick parameters
--------------------------------------------
walk.walkKickVel = {0.06, 0.12} --step / kick / follow 
walk.walkKickSupportMod = {{0,0},{0,0}}
walk.walkKickHeightFactor = 2.0;
walk.tStepWalkKick = 0.30;

walk.sideKickVel1 = {0.04,0.04};
walk.sideKickVel2 = {0.09,0.05};
walk.sideKickVel3 = {0.09,-0.02};
walk.sideKickSupportMod = {{0,0},{0,0}};
walk.tStepSideKick = 0.30;

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
walk.headPitch = 40* math.pi / 180; --Pitch angle offset of OP 
walk.headPitchComp = 0;

local robotName = unix.gethostname();
print(robotName.." walk parameters loaded")
local robotID = 0;

if( robotName=='felix' ) then
	robotID = 8;

	walk.servoBias={0,-6,2,9,12,0,0,0,-6,-5,-2,-1}
	walk.footXComp = -0.009;    
	walk.footYComp = 0.002;  
	walk.kickXComp = -0.010;
	walk.headPitchComp = 4*math.pi/180;
elseif( robotName=='betty' ) then
	robotID = 9;

	walk.servoBias={0,0,2,-6,-1,0,0,0,-13,-1,0,0}
	walk.footXComp = -0.006;    
	walk.footYComp = 0.002;  
	walk.kickXComp = 0.005;
	walk.headPitchComp = 3*math.pi/180;

	--2/6/2012
	walk.servoBias={0,0,2,-6,-1,0,0,0,-3,-1,-10,0}
	walk.footXComp = 0.010;    

elseif( robotName=='linus' ) then
	robotID = 10;

	walk.servoBias={3,1,2,1,1,-3,-8,-3,-13,-4,1,-5}
	walk.footXComp = 0.00;
	walk.kickXComp = -0.004;
	walk.footYComp = 0.0025;  -- 0.04
	walk.headPitchComp = 3*math.pi/180;
elseif( robotName=='lucy' ) then
	robotID = 11;
	walk.servoBias={1,-28,-207,108,-26,-15,-43,-51,110,35,84,-29}; -- new firmware
--	walk.servoBias={29,7,321,16,16,340,15,5,427,10,7,7}
	--  walk.servoBias={-3,1,3,-1,-3,4,1,-3,-9,-1,-8,-1} --4/21, measured by chris
	walk.footXComp = 0.002; 
	walk.footYComp = 0.0020;
	walk.kickXComp = -0.005;

elseif( robotName=='scarface' ) then
	robotID = 5;
	--02/04/12 - Larry V.
	walk.servoBias={0,0,0,0,0,0,0,0,0,-9,-4,0} 
	walk.footXComp = 0.00;
	walk.kickXComp = -0.005;
	walk.footXComp = -0.005;
end

--Apply robot specific compensation to default values
walk.footX = walk.footX + walk.footXComp;
walk.footY = walk.footY + walk.footYComp;
walk.headPitch = walk.headPitch + walk.headPitchComp;

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


