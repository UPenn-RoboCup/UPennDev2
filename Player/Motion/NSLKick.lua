module(..., package.seeall);

require('Body')
require('keyframe')
require('walk')
require('step')
require('vector')
require('Config')
require 'util'

local cwd = unix.getcwd();
if string.find(cwd, "WebotsController") then
  cwd = cwd.."/Player";
end
cwd = cwd.."/Motion"

-- default kick type
kickType = "kickForwardLeft";
active = false;

qLArm0 = Config.kick.qLArm;
qRArm0 = Config.kick.qRArm;
qLArm = vector.new({qLArm0[1],qLArm0[2],qLArm0[3]});
qRArm = vector.new({qRArm0[1],qRArm0[2],qRArm0[3]});

bodyHeight = Config.walk.bodyHeight;
footX = Config.walk.footX+Config.walk.footXComp;
footY = Config.walk.footY+Config.walk.footYComp;
bodyTilt = Config.walk.bodyTilt;
supportX = Config.walk.supportX;
bodyRoll=0;

ankleShift = vector.new({0, 0});
kneeShift=0;
hipShift=vector.new({0,0});
armShift = vector.new({0, 0});

ankleImuParamX=Config.kick.ankleImuParamX;
ankleImuParamY=Config.kick.ankleImuParamY;
kneeImuParamX=Config.kick.kneeImuParamX;
hipImuParamY=Config.kick.hipImuParamY;
armImuParamX=Config.kick.armImuParamX;
armImuParamY=Config.kick.armImuParamX;


qLHipRollCompensation=0;
qRHipRollCompensation=0;
qLHipPitchCompensation=0;
qRHipPitchCompensation=0;

supportCompL=Config.walk.supportCompL;
supportCompR=Config.walk.supportCompR;

hardnessArm=Config.kick.hardnessArm;
hardnessLeg=Config.kick.hardnessLeg;

--kick definitions
kickLeft=Config.kick.kickLeft;
kickRight=Config.kick.kickRight;
kickSideLeft=Config.kick.kickSideLeft;
kickSideRight=Config.kick.kickSideRight;
kickBackLeft=Config.kick.kickBackLeft;
kickBackRight=Config.kick.kickBackRight;
kickSlowSideLeft=Config.kick.kickSlowSideLeft;
kickSlowSideRight=Config.kick.kickSlowSideRight;
avoidLeft=Config.kick.avoidLeft;
avoidRight=Config.kick.avoidRight;


kickState=1;

pTorso = vector.new({0, 0, bodyHeight, 0,bodyTilt,0});
pLLeg=vector.zeros(6);
pRLeg=vector.zeros(6);

torsoShiftX=0;

function entry()
  print("Motion SM:".._NAME.." entry");
  walk.stop();
  walk.zero_velocity();

  -- Reset some values
  qLArm = vector.new({qLArm0[1],qLArm0[2],qLArm0[3]});
  qRArm = vector.new({qRArm0[1],qRArm0[2],qRArm0[3]});
  torsoShiftX=0;  
  
  started = false;
  active = true;
  if kickType=="kickForwardLeft" then 
    kickDef=kickLeft;
  elseif kickType=="kickForwardRight" then 
    kickDef=kickRight;
  elseif kickType=="kickSideLeft" then 
    kickDef=kickSideLeft;
  elseif kickType=="kickSideRight" then 
    kickDef=kickSideRight;
  elseif kickType=="kickSlowSideLeft" then 
    kickDef=kickSlowSideLeft;
  elseif kickType=="kickSlowSideRight" then 
    kickDef=kickSlowSideRight;
  elseif kickType=="kickBackLeft" then 
    kickDef=kickBackLeft;
  elseif kickType=="kickBackRight" then 
    kickDef=kickBackRight;
  elseif kickType=="avoidLeft" then 
    kickDef=avoidLeft;
  elseif kickType=="avoidRight" then 
    kickDef=avoidRight;

  end

  -- disable joint encoder reading
  Body.set_syncread_enable(0);

  uLeft= vector.new({-supportX, footY, 0});
  uRight=vector.new({-supportX, -footY, 0});
  uLeft1= vector.new({-supportX, footY, 0});
  uRight1=vector.new({-supportX, -footY, 0});

  uBody=vector.new({0,0,0});
  uBody0=vector.new({0,0,0});
  uBody1=vector.new({0,0,0});

  zLeft,zRight=0,0;
  zLeft1,zRight1=0,0;
  aLeft,aRight=0,0;
  aLeft1,aRight1=0,0;
  zBody,zBody1=Config.walk.bodyHeight,Config.walk.bodyHeight;
  bodyRoll,bodyRoll1=0,0;

end

function update()

  if (not started and walk.active) then
    walk.update();
    return;
  elseif not started then
    started = true;
    Body.set_head_hardness(.5);
    Body.set_larm_hardness(hardnessArm);
    Body.set_rarm_hardness(hardnessArm);
    Body.set_lleg_hardness(hardnessLeg);
    Body.set_rleg_hardness(hardnessLeg);
    kickState=1;
    t0 = Body.get_time();
  end


  local t = Body.get_time();
  local ph = (t-t0)/kickDef[kickState][2];
  if ph>1 then
    kickState=kickState+1;
    uLeft1[1],uLeft1[2],uLeft1[3]=uLeft[1],uLeft[2],uLeft[3];
    uRight1[1],uRight1[2],uRight1[3]=uRight[1],uRight[2],uRight[3];
    uBody1[1],uBody1[2],uBody1[3]=uBody[1],uBody[2],uBody[3];
	  zLeft1,zRight1=zLeft,zRight;
    aLeft1,aRight1=aLeft,aRight;
    bodyRoll1=bodyRoll;
    zBody1=zBody;

  if kickState>#kickDef then return "done";end

	ph=0;
	t0=Body.get_time();
	if kickType=="kickForwardLeft" or kickType=="kickSideLeft" 
	  or kickType=="kickBackLeft"or kickType=="kickSlowSideLeft"	then 
		Body.set_rleg_slope(16);
		Body.set_lleg_slope(32);
	elseif kickType=="kickForwardRight" or kickType=="kickSideRight" 
	  or kickType=="kickBackRight"	or kickType=="kickSlowSideRight" then 
		Body.set_lleg_slope(16);
		Body.set_rleg_slope(32);
	end
  end

  local kickType=kickDef[kickState][1];

  if kickType==6 then
     torsoShiftX=Config.walk.kickXComp*(1-ph);
  elseif torsoShiftX<Config.walk.kickXComp*0.9 then
     torsoShiftX=Config.walk.kickXComp*ph;
  end


  if kickType==1 or kickType==6 then
	uBody=util.se2_interpolate(ph,uBody1,kickDef[kickState][3]);	
	if #kickDef[kickState]>=4 then
		zBody=ph*kickDef[kickState][4] + (1-ph)*zBody1;
	end
	if #kickDef[kickState]>=5 then
		bodyRoll=ph*kickDef[kickState][5] + (1-ph)*bodyRoll1;
	end
	if kickType==6 then
	  qLArm0 = Config.walk.qLArm;
	  qRArm0 = Config.walk.qRArm;
	  qLArm = vector.new({qLArm0[1],qLArm0[2],qLArm0[3]});
	  qRArm = vector.new({qRArm0[1],qRArm0[2],qRArm0[3]});

	end

  elseif kickType==2 then --Lifting / Landing Left foot
--	uZmp2=kickDef[kickState][3];
	uLeft=util.se2_interpolate(ph,uLeft1,util.pose_global(kickDef[kickState][4],uLeft1));
	zLeft=ph*kickDef[kickState][5] + (1-ph)*zLeft1;
	aLeft=ph*kickDef[kickState][6] + (1-ph)*aLeft1;

	qLHipRollCompensation=0;
	qRHipRollCompensation= -5*math.pi/180;

  elseif kickType==3 then --Lifting / Landing Right foot
--	uZmp2=kickDef[kickState][3];
	uRight=util.se2_interpolate(ph,uRight1,util.pose_global(kickDef[kickState][4],uRight1));
	zRight=ph*kickDef[kickState][5] + (1-ph)*zRight1;
	aRight=ph*kickDef[kickState][6] + (1-ph)*aRight1;

	qLHipRollCompensation= 5*math.pi/180;
	qRHipRollCompensation=0;

  elseif kickType==4 then --Kicking Left foot
	uLeft=util.pose_global(kickDef[kickState][4],uLeft1);
	zLeft=kickDef[kickState][5]
        aLeft=kickDef[kickState][6]

	qLHipRollCompensation=0;
	qRHipRollCompensation=-5*math.pi/180;

  elseif kickType==5 then --Kicking Right foot
	uRight=util.pose_global(kickDef[kickState][4],uRight1);
	zRight=kickDef[kickState][5]
        aRight=kickDef[kickState][6]

	qLHipRollCompensation=5*math.pi/180;
	qRHipRollCompensation=0;

  end

--  pLLeg[1],pLLeg[2],pLLeg[3],pLLeg[5],pLLeg[6]=uLeft[1],uLeft[2],zLeft,aLeft,uLeft[3];
--  pRLeg[1],pRLeg[2],pRLeg[3],pRLeg[5],pRLeg[6]=uRight[1],uRight[2],zRight,aRight,uRight[3];

  uLeftActual=util.pose_global(supportCompL, uLeft);
  uRightActual=util.pose_global(supportCompR, uRight);

  pLLeg[1],pLLeg[2],pLLeg[3],pLLeg[5],pLLeg[6]=uLeftActual[1],uLeftActual[2],zLeft,aLeft,uLeftActual[3];
  pRLeg[1],pRLeg[2],pRLeg[3],pRLeg[5],pRLeg[6]=uRightActual[1],uRightActual[2],zRight,aRight,uRightActual[3];

  uTorso=util.pose_global(vector.new({-footX-torsoShiftX,0,0}),uBody);

  pTorso[1],pTorso[2],pTorso[6]=uTorso[1],uTorso[2],uTorso[3];
  pTorso[3]=zBody;
  pTorso[4]=bodyRoll;
  motion_legs();
  motion_arms();
end

function motion_arms()
  qLArm[1],qLArm[2]=qLArm0[1]+armShift[1],qLArm0[2]+armShift[2];
  qRArm[1],qRArm[2]=qRArm0[1]+armShift[1],qRArm0[2]+armShift[2];

  local uBodyLeft=util.pose_global(uLeft,uBody);
  local uBodyRight=util.pose_global(uRight,uBody);
  local footRel=uBodyLeft[1]-uBodyRight[1];

  local armAngle=math.min(50*math.pi/180,
	math.max(-50*math.pi/180,
	footRel/0.10 * 50*math.pi/180
	));  


  qLArm[1]=qLArm[1]+armAngle;
  qRArm[1]=qRArm[1]-armAngle;
  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);
end

function motion_legs()

--Ankle stabilization using gyro feedback
  imuGyr = Body.get_sensor_imuGyr();

  gyro_roll=imuGyr[1];
  gyro_pitch=imuGyr[2];

  ankleShiftX=procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4]);
  ankleShiftY=procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4]);
  kneeShiftX=procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4]);
  hipShiftY=procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4]);

  armShiftX=procFunc(gyro_pitch*armImuParamX[2],armImuParamX[3],armImuParamX[4]);
  armShiftY=procFunc(gyro_roll*armImuParamY[2],armImuParamY[3],armImuParamY[4]);

  ankleShift[1]=ankleShift[1]+ankleImuParamX[1]*(ankleShiftX-ankleShift[1]);
  ankleShift[2]=ankleShift[2]+ankleImuParamY[1]*(ankleShiftY-ankleShift[2]);
  kneeShift=kneeShift+kneeImuParamX[1]*(kneeShiftX-kneeShift);
  hipShift[2]=hipShift[2]+hipImuParamY[1]*(hipShiftY-hipShift[2]);

  armShift[1]=armShift[1]+armImuParamX[1]*(armShiftX-armShift[1]);
  armShift[2]=armShift[2]+armImuParamY[1]*(armShiftY-armShift[2]);

  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso,0);


  if kicktype==4 then	--Left kick
	  qLegs[10] = qLegs[10] + kneeShift;
	  qLegs[11] = qLegs[11]  + ankleShift[1];
  elseif kicktype==5 then  --Right kick
	  qLegs[4] = qLegs[4] + kneeShift;
	  qLegs[5] = qLegs[5]  + ankleShift[1];
  else
	  qLegs[4] = qLegs[4] + kneeShift;
	  qLegs[5] = qLegs[5]  + ankleShift[1];
	  qLegs[10] = qLegs[10] + kneeShift;
	  qLegs[11] = qLegs[11]  + ankleShift[1];
  end

  qLegs[2] = qLegs[2] + qLHipRollCompensation+hipShift[2];
  qLegs[8] = qLegs[8] + qRHipRollCompensation+hipShift[2];

  qLegs[3] = qLegs[3] + qLHipPitchCompensation;
  qLegs[9] = qLegs[9] + qRHipPitchCompensation;

  qLegs[6] = qLegs[6] + ankleShift[2];
  qLegs[12] = qLegs[12] + ankleShift[2];
  Body.set_lleg_command(qLegs);
end


function exit()
  print("Kick exit");
  kick.active = false;

  Body.set_lleg_slope(32);
  Body.set_rleg_slope(32);

  walk.start();
end

function set_kick(newKick)
    kickType = newKick;
end


function procFunc(a,deadband,maxvalue)
	if a>0 then b=math.min( math.max(0,math.abs(a)-deadband), maxvalue);
	else b=-math.min( math.max(0,math.abs(a)-deadband), maxvalue);
	end
	return b;
end




