module(..., package.seeall);
require('vector')

sit_disable = 1;

stance={};
stance.hardnessLeg = 0.5;
stance.bodyTiltStance=4*math.pi/180; --bodyInitial bodyTilt, 0 for 
stance.dpLimitStance=vector.new({.02, .01, .02, .4, .4, .4});
stance.dpLimitSit=vector.new({.1,.01,.06,.1,.3,.1})*2;
stance.delay = 80; 

-- Head Parameters
head = {};
head.camOffsetZ = 1.30;
head.pitchMin = -55*math.pi/180;
head.pitchMax = 68*math.pi/180;
head.yawMin = -135*math.pi/180;
head.yawMax = 135*math.pi/180;
head.cameraPos = {{0.08, 0, 0}} 
head.cameraAngle = {{0,37.5*math.pi/180,0}}; 
head.neckZ=0.562 --From CoM to neck joint 
head.neckX=0.0; --From CoM to neck joint

--Servo parameters
servo={}
--For charli
servo.idMap={
  22,23,		--Head
  2,4,6,8,		--LArm
  10,12,14,16,18,20,--LLeg
  9,11,13,15,17,19,--RLeg
  1,3,5,7,		--RArm
  21, --Waist
--  25,26, --Aux (hand)
}
--Haven't tested yet
servo.dirReverse={
--  1,2, --Head
--  3,4,5,6, --LArm
  7,8,10,11,12, --LLeg
  13,14,15,18, --RLeg
  19,21,22--RArm
}

----------------------------------------------
--Robot-specific firmware version handling
----------------------------------------------
servo.armBias = vector.new( {-90,0,0,45,    -90,0,0,45} )*math.pi/180; --in radians
servo.pid =1;  --Default new firmware
local robotName = unix.gethostname();
require('calibration');
if calibration.cal and calibration.cal[robotName] then
print('READING FROM CALIBRATION!!!')
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
    --1318,2048,2304,2048, --LArm
    2048,2048,2048,2048, --LArm
    2048,2048,2048,3413,2048,2048, --LLeg
    2048,2048,2048,683,2048,2048, --RLeg
    --2748,2048,1792,2048, --RArm
    2048,2048,2048,2048, --RArm
    2048, --Waist
    2048,2048, --Hands
};

servo.moveRange=vector.new({
    360,360, --Head
    360,360,360,360, --LArm
    360,120,120,120,120,160, --LLeg
    360,120,120,120,120,160, --RLeg
    360,360,360,360, --RArm
    160, --Waist
    360,360, --Hands
})*math.pi/180;

-- PID Parameters
servo.pid_param={
	--Regular PID gain
	{32,0,4},
	--Kick PID gain
	{64,0,4},
};
