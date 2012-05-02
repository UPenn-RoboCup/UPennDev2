module(..., package.seeall);
require('vector')

--Sit/stand stance parameters
stance={};
stance.bodyHeightSit = 0.175;
stance.footXSit = -0.03;
stance.dpLimitSit=vector.new({.03,.01,.06,.1,.3,.1});
stance.bodyHeightDive= 0.25;
stance.bodyTiltStance=20*math.pi/180; --bodyInitial bodyTilt, 0 for webots
stance.dpLimitStance=vector.new({.04, .03, .07, .4, .4, .4});
stance.initangle = {
  0,0,
  105*math.pi/180, 30*math.pi/180, -45*math.pi/180,
  0,  0.055, -0.77, 2.08, -1.31, -0.055, 
  0, -0.055, -0.77, 2.08, -1.31, 0.055,
  105*math.pi/180, -30*math.pi/180, -45*math.pi/180,
}

-- Head Parameters
head = {};
head.camOffsetZ = 1.36;
head.pitchMin = -55*math.pi/180;
head.pitchMax = 68*math.pi/180;
head.yawMin = -90*math.pi/180;
head.yawMax = 90*math.pi/180;
head.cameraPos = {{0.034, 0.0, 0.0332}} --OP, spec value, may need to be recalibrated
head.cameraAngle = {{0.0,0}}; -- We set it zero here
head.neckZ=0.0765; --From CoM to neck joint 
head.neckX=0.013; --From CoM to neck joint

--Servo parameters
servo={}
--For charli
servo.idMap={
  22,23,		--Head
  2,4,6,8,		--LArm
  10,12,14,16,18,20,--LLeg
  9,11,13,15,17,19,--RLeg
  1,3,5,7,		--RArm
  24,25, --Aux (hand)
}
--Haven't tested yet
servo.dirReverse={
  
}

----------------------------------------------
--Robot-specific firmware version handling
----------------------------------------------
servo.armBias = {0,0,0,0,0,0}; --in degree
servo.pid =1;  --Default new firmware
local robotName = unix.gethostname();
require('calibration');
if calibration.cal and calibration.cal[robotName] then
  if calibration.cal[robotName].pid then
    servo.pid = calibration.cal[robotName].pid;
  end
  if calibration.cal[robotName].armBias then
    servo.armBias = calibration.cal[robotName].armBias;
  end
end
-----------------------------------------------

nJoint = #servo.idMap;
-- For new, PID firmware with 14-bit precision
-- Corrected with CHARLI gear reduction ratio
print(robotName.." has 14-bit firmware")
servo.steps=vector.ones(nJoint)*4096;
servo.posZero={
    2048,2048, --Head
    2816,2048,2304,2348, --LArm
    2048,2048,2048,3413,2048,2048, --LLeg
    2048,2048,2048,683,2048,2048, --RLeg
    1280,2048,1792,1748, --RArm
    2048,2048, --Hands
};

servo.moveRange=vector.new({
    360,360, --Head
    360,360,360,360, --LArm
    360,120,120,120,120,160, --LLeg
    360,120,120,120,120,160, --RLeg
    360,360,360,360, --RArm
    360,360, --Hands
})*math.pi/180;

-- PID Parameters
servo.pid_param={
	--Regular PID gain
	{32,0,4},
	--Kick PID gain
	{64,0,4},
};
