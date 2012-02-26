module(..., package.seeall);
require('vector')

--Sitting parameters
sit={};
sit.bodyHeight=0.17+0.05; --Fixed with new kinematics
sit.supportX=-0.010;
sit.bodyTilt=5*math.pi/180;
sit.dpLimit=vector.new({.1,.01,.06,.1,.3,.1});

--Standing parameters
stance={};
stance.dpLimit=vector.new({.04, .03, .04, .4, .4, .4});
stance.dpLimit=vector.new({.04, .03, .07, .4, .4, .4});--Faster standup
--Init angle for start-up
--[[
stance.initangle = vector.new({
  -0.6, -75.8,
  77.9, 0.3, 0.0, -8.8,
  -1.2, -17.5, 42.2, 0.0, 0.0, 0.0,
  -2.9, -3.4, 1.2, -3.2, 0.3, 5.3,
  66.5, -17.9, 0.0, 19.0,}
 ) * math.pi/180;
--]]
stance.initangle = vector.new( { 
 -0.6, -108.5,
 72.9, 1.8, 0.0, -0.3,
 -0.6, -1.0, 0.9, 2.9, -0.3, -0.9,
 0.9, -0.4, -0.0, 1.2, 1.2, 2.9,
 51.6, 0.3, 0.0, 5.6,
 0.0,
}) * math.pi/180;
stance.standangle = vector.new( {
 -1.8, -110.9,
 68.8, 13.5, 0.0, -19.9,
 -8.8, 5.8, 14.1, -1.9, -7.0, -5.6,
 -8.8, -10.1, 15.2, 2.7, -2.6, 11.7,
 65.6, -5.9, 0.0, 17.3,
 29.0,
}) * math.pi/180;
--Servo parameters
servo={}

servo.idMap = {
   20,21, --Head
   2,4,22,6, --Larm
   8,10,12,14,16,18, --LLeg
   7,9,11,13,15,17, --RLegs
   1,3,23,5, --RAram
   19, --Waist
};

nJoint = #servo.idMap;

servo.dirReverse = {
--   2, -- Head
   7,10,11, -- LLeg
   12,13,15,16, --RLeg
   18,19, -- RArm
   23,
};

--Robot-specific firmware version handling
servo.pid = 0; --old firmware default
servo.armBias = {0,0,0,0,0,0}; --in degree
servo.syncread = 1;

local robotName = unix.gethostname();

if servo.pid ==0 then -- For old firmware with 12-bit precision
  print(robotName.." has 12-bit firmware")
  servo.steps = {
    1024,1024,
    1024,1024,1024,1024,
    1024,4096,1024,4096,1024,1024,
    1024,4096,1024,4096,1024,1024,
    1024,1024,1024,1024,
    1024,
  }
  servo.moveRange = vector.new({
    300,300,
    300,300,300,300,
    300,251,300,251,300,300,
    300,251,300,251,300,300,
    300,300,300,300,
    300,
  })*math.pi/180;
--]]
  servo.posZero={
    512,512,
    512,512,512,512,
    511,1650,450,1650,580,508,
    508,2440,490,2100,578,360,
    512,512,512,512,
    512, --For waist
  }

else -- For new, PID firmware with 14-bit precision
  print(robotName.." has 14-bit firmware")
  servo.steps=vector.ones(nJoint)*4096;
  servo.posZero={
	2048,2048, --Head
	1024,2560,3072, --LArm
	2048,2048,2048,2048,2048,2048, --LLeg
	2048,2048,2048,2048,2048,2048, --RLeg
	3072,1536,1024, --RArm
  };
  servo.moveRange=vector.ones(nJoint)*360*math.pi/180;
end

--Lucy specific arm installation bias
if( robotName=='lucy' ) then
  servo.armBias = vector.new({0,20,0,0,0,-20})*math.pi/180*servo.steps[1]/servo.moveRange[1];
end

--Measured IMU bias parameters

gyro={};
gyro.rpy={2,3,1}	--axis remap, rotation in x,y,z
acc={};
acc.xyz={2,1,3};	--axis remap

angle={};
angle.gMax = 1.3;
angle.gMin= 0.7;
angle.accFactor=0.2;

-- OP Spec, 0.0008 V/dps  / (1.5/512) V/step
-- Changed for HP... TODO: double check
gyro.sensitivity=vector.new({-1,-1,1})/0.02435
gyro.zero=vector.new({499,495,496});

--Those biases can be measured using test_imu.lua
acc.sensitivity=vector.new({1,-1,-1})/110; --Measured value
acc.zero=vector.new({512,512,512}); --Measured value

-- Head Parameters

head = {};
head.camOffsetZ = 0.37;
head.pitchMin = -55*math.pi/180;
head.pitchMax = 68*math.pi/180;
head.yawMin = -90*math.pi/180;
head.yawMax = 90*math.pi/180;

head.cameraPos = {{0.034, 0.0, 0.0332}} --OP, spec value, may need to be recalibrated
head.cameraAngle = {{0.0,0}}; -- We set it zero here
head.neckZ=0.0765; --From CoM to neck joint 
head.neckX=0.013; --From CoM to neck joint
head.linkParam = {1.0242,-0.6363};
head.invlinkParam={1.5839,0.0397};

