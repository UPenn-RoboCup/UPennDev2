module(..., package.seeall);
require('vector')

--Sitting parameters
sit={};
sit.bodyHeight=0.17+0.05; --Fixed with new kinematics
sit.supportX=-0.010;
sit.bodyTilt=5*math.pi/180;
sit.dpLimit=vector.new({.1,.01,.06,.1,.3,.1});

--Init angle for start-up
sit.initangle = {
	0,0,
	105*math.pi/180, 30*math.pi/180, -45*math.pi/180,
	0,  0.055, -0.77, 2.08, -1.31, -0.055, 
	0, -0.055, -0.77, 2.08, -1.31, 0.055,
	105*math.pi/180, -30*math.pi/180, -45*math.pi/180,
}

--Standing parameters
stance={};
stance.dpLimit=vector.new({.04, .03, .04, .4, .4, .4});
stance.dpLimit=vector.new({.04, .03, .07, .4, .4, .4});--Faster standup

--Servo parameters

servo={}

servo.idMap={
	19,20,		--Head
	2,4,6,		--LArm
	8,10,12,14,16,18,--LLeg
	7,9,11,13,15,17,--RLeg
	1,3,5,		--RArm
}

nJoint = #servo.idMap;

servo.dirReverse={
	2,	--Head
	4,	--LArm
	6,7,8,9,--LLeg
	12,13,16,--RLeg
	18,19,20,--RArm
}

--Robot-specific firmware version handling
servo.pid = 0; --old firmware default
servo.armBias = {0,0,0,0,0,0}; --in degree

local robotName = unix.gethostname();
if( robotName=='felix' ) then
elseif( robotName=='betty' ) then
elseif( robotName=='linus' ) then
elseif( robotName=='lucy' ) then
	servo.pid = 1;
elseif( robotName=='scarface' ) then
end

if servo.pid ==0 then -- For old firmware with 12-bit precision
	print(robotName.." has 12-bit firmware")
	servo.steps=vector.ones(nJoint)*1024;
	servo.moveRange=vector.ones(nJoint)*300*math.pi/180;
	servo.posZero={
		512,512,
		205,665,819,
		512,512,512,512,512,512,
		512,512,512,512,512,512,
		819,358,205,
		512,		--For aux
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
	servo.idMap = {
		19,20,		--Head
		2,4,6,		--LArm
		8,10,12,14,16,18,--LLeg
		7,9,11,13,15,17,--RLeg
		1,3,5,		--RArm
		21, 		--Aux servo
	}
end

--Measured IMU bias parameters

gyro={};
gyro.rpy={3,2,1}	--axis remap, rotation in x,y,z
acc={};
acc.xyz={2,1,3};	--axis remap

angle={};
angle.gMax = 1.3;
angle.gMin= 0.7;
angle.accFactor=0.2;

gyro.sensitivity=vector.new({1,1,1})/0.273 -- Spec, 0.0008 V/dps  / (1.5/512) V/step 
gyro.zero=vector.new({512,512,512});

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

