--CHARLI WALK CODE

module(..., package.seeall);

require('Body')
require('Kinematics')
require('Config');
require('vector')
require('mcm')
require('unix')
require('util')

-- Walk Parameters
-- Stance and velocity limit values
stanceLimitX=Config.walk.stanceLimitX or {-0.10 , 0.10};
stanceLimitY=Config.walk.stanceLimitY or {0.09 , 0.20};
stanceLimitA=Config.walk.stanceLimitA or {-0*math.pi/180, 40*math.pi/180};
velLimitX = Config.walk.velLimitX or {-.06, .08};
velLimitY = Config.walk.velLimitY or {-.06, .06};
velLimitA = Config.walk.velLimitA or {-.4, .4};
velDelta = Config.walk.velDelta or {.03,.015,.15};
vaFactor = Config.walk.vaFactor or 0.6;

--Toe/heel overlap checking values
footSizeX = Config.walk.footSizeX or {-0.05,0.05};
stanceLimitMarginY = Config.walk.stanceLimitMarginY or 0.015;
stanceLimitY2= 2* Config.walk.footY-stanceLimitMarginY;

--OP default stance width: 0.0375*2 = 0.075
--Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
--Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

--Stance parameters
bodyHeight = Config.walk.bodyHeight;
bodyTilt=Config.walk.bodyTilt or 0;
footX = mcm.get_footX();
footY = Config.walk.footY;
supportX = Config.walk.supportX;
supportY = Config.walk.supportY;
qLArm0=Config.walk.qLArm;
qRArm0=Config.walk.qRArm;
qLArmKick0=Config.walk.qLArmKick;
qRArmKick0=Config.walk.qRArmKick;

--Hardness parameters
hardnessSupport = Config.walk.hardnessSupport or 0.7;
hardnessSwing = Config.walk.hardnessSwing or 0.5;
hardnessArm = Config.walk.hardnessArm or 0.2;

--Gait parameters
tStep0 = Config.walk.tStep;
tStep = Config.walk.tStep;
tZmp = Config.walk.tZmp;
stepHeight0 = Config.walk.stepHeight;
stepHeight = Config.walk.stepHeight;
ph1Single = Config.walk.phSingle[1];
ph2Single = Config.walk.phSingle[2];
ph1Zmp,ph2Zmp=ph1Single,ph2Single;

--Compensation parameters
hipRollCompensation = Config.walk.hipRollCompensation;
hipPitchCompensation = Config.walk.hipPitchCompensation or 0;
ankleMod = Config.walk.ankleMod or {0,0};
spreadComp = Config.walk.spreadComp or 0;
turnCompThreshold = Config.walk.turnCompThreshold or 0;
turnComp = Config.walk.turnComp or 0;

--Gyro stabilization parameters
ankleImuParamX = Config.walk.ankleImuParamX;
ankleImuParamY = Config.walk.ankleImuParamY;
kneeImuParamX = Config.walk.kneeImuParamX;
hipImuParamY = Config.walk.hipImuParamY;
armImuParamX = Config.walk.armImuParamX;
armImuParamY = Config.walk.armImuParamY;

--Support bias parameters to reduce backlash-based instability
velFastForward = Config.walk.velFastForward or 0.06;
supportFront = Config.walk.supportFront or 0;
supportFront2 = Config.walk.supportFront2 or 0;
supportBack = Config.walk.supportBack or 0;
supportSideX = Config.walk.supportSideX or 0;
supportSideY = Config.walk.supportSideY or 0;

--Initial body swing 
supportModYInitial = Config.walk.supportModYInitial or 0;

--WalkKick parameters
walkKickDef = Config.walk.walkKickDef;
walkKickPh = Config.walk.walkKickPh;
toeTipCompensation = 0;

use_alternative_trajectory = Config.walk.use_alternative_trajectory or 0;


----------------------------------------------------------
-- Walk state variables
----------------------------------------------------------

uTorso = vector.new({supportX, 0, 0});
uLeft = vector.new({0, footY, 0});
uRight = vector.new({0, -footY, 0});

pLLeg = vector.new({0, footY, 0, 0,0,0});
pRLeg = vector.new({0, -footY, 0, 0,0,0});
pTorso = vector.new({supportX, 0, bodyHeight, 0,bodyTilt,0});

velCurrent = vector.new({0, 0, 0});
velCommand = vector.new({0, 0, 0});
velDiff = vector.new({0, 0, 0});

--ZMP exponential coefficients:
aXP, aXN, aYP, aYN = 0, 0, 0, 0;

--Gyro stabilization variables
ankleShift = vector.new({0, 0});
kneeShift = 0;
hipShift = vector.new({0,0});
armShift = vector.new({0, 0});

active = true;
started = false;
iStep0 = -1;
iStep = 0;
t0 = Body.get_time();
tLastStep = Body.get_time();
ph0=0;ph=0;

stopRequest = 2;
canWalkKick = 1; --Can we do walkkick with this walk code?
walkKickRequest = 0; 
walkKick = walkKickDef["FrontLeft"];
current_step_type = 0;

initial_step=2;


testing = Config.walk.testing or false
----------------------------------------------------------
-- End initialization 
----------------------------------------------------------

function entry()
  print ("Motion: Walk entry")
  --SJ: now we always assume that we start walking with feet together
  --Because joint readings are not always available with darwins
  uLeft = util.pose_global(vector.new({-supportX, footY, 0}),uTorso);
  uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso);

  uLeft1, uLeft2 = uLeft, uLeft;
  uRight1, uRight2 = uRight, uRight;
  uTorso1, uTorso2 = uTorso, uTorso;
  uSupport = uTorso;

  --Place arms in appropriate position at sides
  Body.set_larm_command(qLArm0);
  Body.set_larm_hardness(hardnessArm);
  Body.set_rarm_command(qRArm0);
  Body.set_rarm_hardness(hardnessArm);

  walkKickRequest = 0;
end


function update()

  if testing then
    --Stance parameters
    bodyHeight = Config.walk.bodyHeight;
    bodyTilt=Config.walk.bodyTilt or 0;
    footX = mcm.get_footX();
    footY = Config.walk.footY;
    supportX = Config.walk.supportX;
    supportY = Config.walk.supportY;

    tStep0 = Config.walk.tStep;
    tStep = Config.walk.tStep;
    tZmp = Config.walk.tZmp;
    stepHeight0 = Config.walk.stepHeight;
    stepHeight = Config.walk.stepHeight;
    ph1Single = Config.walk.phSingle[1];
    ph2Single = Config.walk.phSingle[2];
    ph1Zmp,ph2Zmp=ph1Single,ph2Single;

    velLimitX = Config.walk.velLimitX or {-.06, .08};
    velLimitY = Config.walk.velLimitY or {-.06, .06};
    velLimitA = Config.walk.velLimitA or {-.4, .4};
    velDelta = Config.walk.velDelta or {.03,.015,.15};
  end

  footX = mcm.get_footX();

  t = Body.get_time();

  --Don't run update if the robot is sitting or standing
  bodyHeightCurrent = vcm.get_camera_bodyHeight();
  if  bodyHeightCurrent<bodyHeight-0.01 then
    return;
  end
   if (not active) then 
    update_still();
    return; 
  end

  if not started then
    started=true;
    tLastStep = Body.get_time();
  end
  ph0=ph;
  
  --SJ: Variable tStep support for walkkick
  ph = (t-tLastStep)/tStep;
  if ph>1 then
    iStep=iStep+1;
    ph=ph-math.floor(ph);
    tLastStep=tLastStep+tStep;
  end

  --Stop when stopping sequence is done
  if (iStep > iStep0) and(stopRequest==2) then
      stopRequest = 0;
      active = false;
      return "stop";
  end

  -- New step
  if (iStep > iStep0) then
    update_velocity();
    iStep0 = iStep;
    supportLeg = iStep % 2; -- 0 for left support, 1 for right support
    uLeft1 = uLeft2;
    uRight1 = uRight2;
    uTorso1 = uTorso2;

    supportMod = {0,0}; --Support Point modulation for walkkick
    shiftFactor = 0.5; --How much should we shift final Torso pose?


    check_walkkick(); 

    if walkKickRequest==0 then
      if (stopRequest==1) then  --Final step
        stopRequest=2;
        velCurrent=vector.new({0,0,0});
        velCommand=vector.new({0,0,0});
        if supportLeg == 0 then        -- Left support
          uRight2 = util.pose_global({0,-2*footY,0}, uLeft1);
        else        -- Right support
          uLeft2 = util.pose_global({0,2*footY,0}, uRight1);
        end
      else --Normal walk, advance steps
        tStep=tStep0; 
        if supportLeg == 0 then-- Left support
          uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
        else  -- Right support
          uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
        end
        --Velocity-based support point modulation
        toeTipCompensation = 0;
        if velDiff[1]>0 then --Accelerating to front
	        supportMod[1] = supportFront2;
        elseif velCurrent[1]>velFastForward then
	        supportMod[1] = supportFront;
          toeTipCompensation = ankleMod[1];
        elseif velCurrent[1]<0 then
          supportMod[1] = supportBack; 
        end
        if velCurrent[2]>0.015 then
          supportMod[1] = supportSideX; 
          supportMod[2] = supportSideY; 
        elseif velCurrent[2]<-0.015 then
          supportMod[1] = supportSideX; 
          supportMod[2] = -supportSideY; 
        end
      end
    end

    uTorso2 = step_torso(uLeft2, uRight2,shiftFactor);

    --Adjustable initial step body swing
    if initial_step>0 then 
      if supportLeg == 0 then --LS
        supportMod[2]=supportModYInitial;
      else --RS
	supportMod[2]=-supportModYInitial;
      end
    end

    --Apply velocity-based support point modulation for uSupport
    if supportLeg == 0 then --LS
	local uLeftTorso = util.pose_relative(uLeft1,uTorso1);
        local uTorsoModded = util.pose_global(
	  vector.new({supportMod[1],supportMod[2],0}),uTorso);
	local uLeftModded = util.pose_global (uLeftTorso,uTorsoModded); 
        uSupport = util.pose_global(
	  {supportX, supportY, 0},uLeftModded);
        Body.set_lleg_hardness(hardnessSupport);
        Body.set_rleg_hardness(hardnessSwing);
    else --RS
	local uRightTorso = util.pose_relative(uRight1,uTorso1);
        local uTorsoModded = util.pose_global(
	  vector.new({supportMod[1],supportMod[2],0}),uTorso);
	local uRightModded = util.pose_global (uRightTorso,uTorsoModded); 
        uSupport = util.pose_global(
	  {supportX, -supportY, 0}, uRightModded);
        Body.set_lleg_hardness(hardnessSwing);
        Body.set_rleg_hardness(hardnessSupport);
    end

    --Compute ZMP coefficients
    m1X = (uSupport[1]-uTorso1[1])/(tStep*ph1Zmp);
    m2X = (uTorso2[1]-uSupport[1])/(tStep*(1-ph2Zmp));
    m1Y = (uSupport[2]-uTorso1[2])/(tStep*ph1Zmp);
    m2Y = (uTorso2[2]-uSupport[2])/(tStep*(1-ph2Zmp));
    aXP, aXN = zmp_solve(uSupport[1], uTorso1[1], uTorso2[1],
                          uTorso1[1], uTorso2[1]);
    aYP, aYN = zmp_solve(uSupport[2], uTorso1[2], uTorso2[2],
                          uTorso1[2], uTorso2[2]);

    --Compute maximum COM speed
--[[
    dy0=(aYP-aYN)/tZmp + m1Y* (1-math.cosh(ph1Zmp*tStep/tZmp));
    print("max DY:",dy0);
--]]

  end --End new step
  
  xFoot, zFoot = foot_phase(ph);  
  if initial_step>0 then zFoot=0;  end --Don't lift foot at initial step
  pLLeg[3], pRLeg[3] = 0;
  if supportLeg == 0 then    -- Left support
    if current_step_type>1 then --walkkick
      if xFoot<walkKickPh then uRight = 
		util.se2_interpolate(xFoot*2, uRight1, uRight15);
      else uRight = util.se2_interpolate(xFoot*2-1, uRight15, uRight2);
      end
    else
      uRight = util.se2_interpolate(xFoot, uRight1, uRight2);
    end
    pRLeg[3] = stepHeight*zFoot;
  else    -- Right support
    if current_step_type>1 then --walkkick
      if xFoot<walkKickPh then uLeft = util.se2_interpolate(xFoot*2, uLeft1, uLeft15);
      else uLeft = util.se2_interpolate(xFoot*2-1, uLeft15, uLeft2);      
      end
    else
      uLeft = util.se2_interpolate(xFoot, uLeft1, uLeft2);
    end
    pLLeg[3] = stepHeight*zFoot;
  end
  uTorsoOld=uTorso;

  uTorso = zmp_com(ph);

--Leg spread compensation
  local spread=util.mod_angle((uLeft[3]-uRight[3])/2);
  local spreadCompX = spreadComp * (1-math.cos(spread));

  local turnCompX=0;
  if math.abs(velCurrent[3])>turnCompThreshold then
    turnCompX = turnComp;
  end

  uTorsoActual = util.pose_global(
	vector.new({-footX+spreadCompX+turnCompX,0,0}),uTorso);

--  uTorsoActual = util.pose_global(vector.new({-footX,0,0}),uTorso);

  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  pTorso[1], pTorso[2], pTorso[6] = uTorsoActual[1], uTorsoActual[2], uTorsoActual[3];

  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
  motion_legs(qLegs);
  motion_arms();
end

function check_walkkick()
  --Walkkick def: 
  --tStep stepType supportFoot stepHeight bodyPosMod footPos1 footPos2
  if walkKickRequest==0 then return; end

  if walkKickRequest>0 and
    walkKickRequest>#walkKick then
    print("NEWNEWNEWKICK: WALKKICK DONE");
    walkKickRequest = 0;
    tStep = tStep0;
    stepHeight = stepHeight0;
    current_step_type=0;
    return;
  end

  if walkKickRequest ==1 then 
    --Check current supporLeg and feet positions
    --and advance steps until ready
    uFootErr = util.pose_relative(uLeft1,
			util.pose_global({0,2*footY,0},uRight1) );
    if supportLeg~=walkKick[1][3] or 
         math.abs(uFootErr[1])>0.02 or
	  math.abs(uFootErr[2])>0.01 or
	  math.abs(uFootErr[3])>10*math.pi/180 then
      if supportLeg == 0 then uRight2 = util.pose_global({0,-2*footY,0}, uLeft1); 
      else uLeft2 = util.pose_global({0,2*footY,0}, uRight1); 
      end
      return;
    end
  end
--  print("NEWNEWNEWKICK: WALKKICK, count",walkKickRequest);

  tStep = walkKick[walkKickRequest][1];   
  current_step_type = walkKick[walkKickRequest][2];   
  supportLeg = walkKick[walkKickRequest][3];
  stepHeight = walkKick[walkKickRequest][4];
  supportMod = walkKick[walkKickRequest][5];

  shiftFactor = walkKick[walkKickRequest][6];

  if #walkKick[walkKickRequest] <=7 then
    footPos1 =  walkKick[walkKickRequest][7];
    if supportLeg == 0 then
      uRight2 = util.pose_global(
 	      {footPos1[1],footPos1[2]-2*footY,footPos1[3]},uLeft1);
    else
      uLeft2 = util.pose_global(
 	      {footPos1[1],footPos1[2]+2*footY,footPos1[3]},uRight1);
    end
  else
    footPos1 =  walkKick[walkKickRequest][7];
    footPos2 =  walkKick[walkKickRequest][8];
    if supportLeg == 0 then
      uRight15 = util.pose_global(
 	      {footPos1[1],footPos1[2]-2*footY,footPos1[3]},uLeft1);
      uRight2 = util.pose_global(
 	      {footPos2[1],footPos2[2]-2*footY,footPos2[3]},uLeft1);
    else
      uLeft15 = util.pose_global(
 	      {footPos1[1],footPos1[2]+2*footY,footPos1[3]},uRight1);
      uLeft2 = util.pose_global(
 	      {footPos2[1],footPos2[2]+2*footY,footPos2[3]},uRight1);
    end
  end

  walkKickRequest = walkKickRequest + 1;
end


function update_still()
  uTorso = step_torso(uLeft, uRight,0.5);
  uTorsoActual = util.pose_global(vector.new({-footX,0,0}),uTorso);
  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  pLLeg[3], pRLeg[3] = 0;
  pTorso[1], pTorso[2], pTorso[6] = uTorsoActual[1], uTorsoActual[2], uTorsoActual[3];
  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
  motion_legs(qLegs);
  motion_arms();
end


function motion_legs(qLegs)
  phComp = math.min(1, phSingle/.1, (1-phSingle)/.1);

  --Ankle stabilization using gyro feedback
  imuGyr = Body.get_sensor_imuGyrRPY();

  gyro_roll0=imuGyr[1];
  gyro_pitch0=imuGyr[2];

  --get effective gyro angle considering body angle offset
  if not active then --double support
    yawAngle = (uLeft[3]+uRight[3])/2-uTorsoActual[3];
  elseif supportLeg == 0 then  -- Left support
    yawAngle = uLeft[3]-uTorsoActual[3];
  elseif supportLeg==1 then
    yawAngle = uRight[3]-uTorsoActual[3];
  end
  gyro_roll = gyro_roll0*math.cos(yawAngle) +
    -gyro_pitch0* math.sin(yawAngle);
  gyro_pitch = gyro_pitch0*math.cos(yawAngle)
    -gyro_roll0* math.sin(yawAngle);

  ankleShiftX=util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4]);
  ankleShiftY=util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4]);
  kneeShiftX=util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4]);
  hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4]);
  armShiftX=util.procFunc(gyro_pitch*armImuParamY[2],armImuParamY[3],armImuParamY[4]);
  armShiftY=util.procFunc(gyro_roll*armImuParamY[2],armImuParamY[3],armImuParamY[4]);

  ankleShift[1]=ankleShift[1]+ankleImuParamX[1]*(ankleShiftX-ankleShift[1]);
  ankleShift[2]=ankleShift[2]+ankleImuParamY[1]*(ankleShiftY-ankleShift[2]);
  kneeShift=kneeShift+kneeImuParamX[1]*(kneeShiftX-kneeShift);
  hipShift[2]=hipShift[2]+hipImuParamY[1]*(hipShiftY-hipShift[2]);
  armShift[1]=armShift[1]+armImuParamX[1]*(armShiftX-armShift[1]);
  armShift[2]=armShift[2]+armImuParamY[1]*(armShiftY-armShift[2]);

  --TODO: Toe/heel lifting

hipPitchCompensationL = Config.walk.hipPitchCompensationL or 
-2*math.pi/180;
hipPitchCompensationR = Config.walk.hipPitchCompensationR or 
-0.5*math.pi/180;

  if not active then --Double support, standing still
    --qLegs[2] = qLegs[2] + hipShift[2];    --Hip roll stabilization
    qLegs[4] = qLegs[4] + kneeShift;    --Knee pitch stabilization
    qLegs[5] = qLegs[5]  + ankleShift[1];    --Ankle pitch stabilization
    --qLegs[6] = qLegs[6] + ankleShift[2];    --Ankle roll stabilization

    --qLegs[8] = qLegs[8]  + hipShift[2];    --Hip roll stabilization
    qLegs[10] = qLegs[10] + kneeShift;    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1];    --Ankle pitch stabilization
    --qLegs[12] = qLegs[12] + ankleShift[2];    --Ankle roll stabilization

  elseif supportLeg == 0 then  -- Left support
    qLegs[2] = qLegs[2] + hipShift[2];    --Hip roll stabilization
    qLegs[4] = qLegs[4] + kneeShift;    --Knee pitch stabilization
    qLegs[5] = qLegs[5]  + ankleShift[1];    --Ankle pitch stabilization
    qLegs[6] = qLegs[6] + ankleShift[2];    --Ankle roll stabilization

    qLegs[11] = qLegs[11]  + toeTipCompensation*phComp;--Lifting toetip

    qLegs[2] = qLegs[2] + hipRollCompensation*phComp; --Hip roll 
    qLegs[3] = qLegs[3] + hipPitchCompensationL*phComp; 

  else
    qLegs[8] = qLegs[8]  + hipShift[2];    --Hip roll stabilization
    qLegs[10] = qLegs[10] + kneeShift;    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1];    --Ankle pitch stabilization
    qLegs[12] = qLegs[12] + ankleShift[2];    --Ankle roll stabilization

    qLegs[5] = qLegs[5]  + toeTipCompensation*phComp;--Lifting toetip



    qLegs[8] = qLegs[8] - hipRollCompensation*phComp;--Hip roll 
    qLegs[9] = qLegs[9] + hipPitchCompensationR*phComp; 

  end

  Body.set_lleg_command(qLegs);
end

function motion_arms()
  local qLArmActual={};   
  local qRArmActual={};   

  if current_step_type== 3 then --Side kick, wide arm stance
    qLArmActual[1],qLArmActual[2]=qLArmKick0[1]+armShift[1],qLArmKick0[2]+armShift[2];
    qRArmActual[1],qRArmActual[2]=qRArmKick0[1]+armShift[1],qRArmKick0[2]+armShift[2];
  else --Normal arm stance
    qLArmActual[1],qLArmActual[2]=qLArm0[1]+armShift[1],qLArm0[2]+armShift[2];
    qRArmActual[1],qRArmActual[2]=qRArm0[1]+armShift[1],qRArm0[2]+armShift[2];
  end

  qLArmActual[2]=math.max(8*math.pi/180,qLArmActual[2])
  qRArmActual[2]=math.min(-8*math.pi/180,qRArmActual[2]);
  qLArmActual[3]=qLArm0[3];
  qRArmActual[3]=qRArm0[3];
  Body.set_larm_command(qLArmActual);
  Body.set_rarm_command(qRArmActual);
end

function exit()
end

function step_left_destination(vel, uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0);
  local u2 = util.pose_global(.5*vel, u1);
  local uLeftPredict = util.pose_global({0, footY, 0}, u2);
  local uLeftRight = util.pose_relative(uLeftPredict, uRight);
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap= -footSizeX[1]*uLeftRight[3];
  local heelOverlap= -footSizeX[2]*uLeftRight[3];
  local limitY = math.max(stanceLimitY[1],
	stanceLimitY2+math.max(toeOverlap,heelOverlap));

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2]);
  uLeftRight[2] = math.min(math.max(uLeftRight[2], limitY),stanceLimitY[2]);
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2]);

  return util.pose_global(uLeftRight, uRight);
end

function step_right_destination(vel, uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0);
  local u2 = util.pose_global(.5*vel, u1);
  local uRightPredict = util.pose_global({0, -footY, 0}, u2);
  local uRightLeft = util.pose_relative(uRightPredict, uLeft);
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap= footSizeX[1]*uRightLeft[3];
  local heelOverlap= footSizeX[2]*uRightLeft[3];
  local limitY = math.max(stanceLimitY[1],
	stanceLimitY2+math.max(toeOverlap,heelOverlap));

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2]);
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -limitY);
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1]);

  return util.pose_global(uRightLeft, uLeft);
end

function step_torso(uLeft, uRight,shiftFactor)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeft);
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight);
  return util.se2_interpolate(shiftFactor, uLeftSupport, uRightSupport);
end

function set_velocity(vx, vy, va)
  --Filter the commanded speed
  vx= math.min(math.max(vx,velLimitX[1]),velLimitX[2]);
  vy= math.min(math.max(vy,velLimitY[1]),velLimitY[2]);
  va= math.min(math.max(va,velLimitA[1]),velLimitA[2]);

  --Slow down when turning
  vFactor = 1-math.abs(va)/vaFactor;

  local stepMag=math.sqrt(vx^2+vy^2);
  local magFactor=math.min(velLimitX[2]*vFactor,stepMag)/(stepMag+0.000001);

  velCommand[1]=vx*magFactor;
  velCommand[2]=vy*magFactor;
  velCommand[3]=va;

  velCommand[1] = math.min(math.max(velCommand[1],velLimitX[1]),velLimitX[2]);
  velCommand[2] = math.min(math.max(velCommand[2],velLimitY[1]),velLimitY[2]);
  velCommand[3] = math.min(math.max(velCommand[3],velLimitA[1]),velLimitA[2]);

end

function update_velocity()
  velDiff[1]= math.min(math.max(velCommand[1]-velCurrent[1],
	-velDelta[1]),velDelta[1]);
  velDiff[2]= math.min(math.max(velCommand[2]-velCurrent[2],
	-velDelta[2]),velDelta[2]);
  velDiff[3]= math.min(math.max(velCommand[3]-velCurrent[3],
	-velDelta[3]),velDelta[3]);

  velCurrent[1] = velCurrent[1]+velDiff[1];
  velCurrent[2] = velCurrent[2]+velDiff[2];
  velCurrent[3] = velCurrent[3]+velDiff[3];

  if initial_step>0 then
     velCurrent=vector.new({0,0,0})
     initial_step=initial_step-1;
  end
end

function get_velocity()
  return velCurrent;
end

function start()
  stopRequest = 0;
  if (not active) then
    active = true;
    started = false;
    iStep0 = -1;
    t0 = Body.get_time();
    tLastStep = Body.get_time();
    initial_step=2;
  end
end

function stop()
  --Always stops with feet together (which helps kicking)
  stopRequest = math.max(1,stopRequest);
  --  stopRequest = 2; --Stop w/o feet together
end

function stopAlign() --Depreciated, we always stop with feet together 
  stop()
end

function doWalkKickLeft()
  if walkKickRequest==0 then
    walkKickRequest = 1; 
    walkKick = walkKickDef["FrontLeft"];
  end
end

function doWalkKickRight()
 if walkKickRequest==0 then
    walkKickRequest = 1; 
    walkKick = walkKickDef["FrontRight"];
  end
end

function doSideKickLeft()
 if walkKickRequest==0 then
    walkKickRequest = 1; 
    walkKick = walkKickDef["SideLeft"];
  end
end

function doSideKickRight()
 if walkKickRequest==0 then
    walkKickRequest = 1; 
    walkKick = walkKickDef["SideRight"];
  end
end

--dummy function for NSL kick, depreciated
function zero_velocity()
end

function doPunch(punchtype)
end

function stance_reset() --standup/sitdown/falldown handling
end

function switch_stance(stance)
end

function get_odometry(u0)
  if (not u0) then
    u0 = vector.new({0, 0, 0});
  end
  local uFoot = util.se2_interpolate(.5, uLeft, uRight);
  return util.pose_relative(uFoot, u0), uFoot;
end

function get_body_offset()
  local uFoot = util.se2_interpolate(.5, uLeft, uRight);
  return util.pose_relative(uTorso, uFoot);
end

function zmp_solve(zs, z1, z2, x1, x2)
  --[[
    Solves ZMP equation:
    x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
    where the ZMP point is piecewise linear:
    z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
  --]]
  local T1 = tStep*ph1Zmp;
  local T2 = tStep*ph2Zmp;
  local m1 = (zs-z1)/T1;
  local m2 = -(zs-z2)/(tStep-T2);

  local c1 = x1-z1+tZmp*m1*math.sinh(-T1/tZmp);
  local c2 = x2-z2+tZmp*m2*math.sinh((tStep-T2)/tZmp);
  local expTStep = math.exp(tStep/tZmp);
  local aP = (c2 - c1/expTStep)/(expTStep-1/expTStep);
  local aN = (c1*expTStep - c2)/(expTStep-1/expTStep);
  return aP, aN;
end

--Finds the necessary COM for stability and returns it
function zmp_com(ph)
  local com = vector.new({0, 0, 0});
  expT = math.exp(tStep*ph/tZmp);
  com[1] = uSupport[1] + aXP*expT + aXN/expT;
  com[2] = uSupport[2] + aYP*expT + aYN/expT;
  if (ph < ph1Zmp) then
    com[1] = com[1] + m1X*tStep*(ph-ph1Zmp)
              -tZmp*m1X*math.sinh(tStep*(ph-ph1Zmp)/tZmp);
    com[2] = com[2] + m1Y*tStep*(ph-ph1Zmp)
              -tZmp*m1Y*math.sinh(tStep*(ph-ph1Zmp)/tZmp);
  elseif (ph > ph2Zmp) then
    com[1] = com[1] + m2X*tStep*(ph-ph2Zmp)
              -tZmp*m2X*math.sinh(tStep*(ph-ph2Zmp)/tZmp);
    com[2] = com[2] + m2Y*tStep*(ph-ph2Zmp)
              -tZmp*m2Y*math.sinh(tStep*(ph-ph2Zmp)/tZmp);
  end
  --com[3] = .5*(uLeft[3] + uRight[3]);
  --Linear speed turning
  com[3] = ph* (uLeft2[3]+uRight2[3])/2 + (1-ph)* (uLeft1[3]+uRight1[3])/2;
  return com;
end

function foot_phase(ph)
  -- Computes relative x,z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1);
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew));
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew));


  use_alternative_trajectory = 1;
  if use_alternative_trajectory>0 then
    ph1FootPhase = 0.1;
    ph2FootPhase = 0.4;
    ph3FootPhase = 0.7;
 
    exp1FootPhase = 2;
    exp2FootPhase = 2;
    exp3FootPhase = 2;

    zFootLand = 0.2;    

    if phSingle < ph1FootPhase then
      phZTemp = phSingle / ph2FootPhase;
      fxf = 0;
      fzf = 1 - (1-phZTemp)^exp1FootPhase;
    elseif phSingle < ph2FootPhase then
      phXTemp = (phSingle-ph1FootPhase)/(ph3FootPhase-ph1FootPhase);
      phZTemp = phSingle / ph2FootPhase;
      fxf =  .5*(1-math.cos(math.pi*phXTemp));
      fzf = 1 - (1-phZTemp)^exp1FootPhase;
    elseif phSingle < ph3FootPhase then
      phXTemp = (phSingle-ph1FootPhase)/(ph3FootPhase-ph1FootPhase);
      phZTemp = (phSingle-ph2FootPhase)/(ph3FootPhase-ph2FootPhase);
      fxf =  .5*(1-math.cos(math.pi*phXTemp));
      fzf = 1 - phZTemp^exp2FootPhase*(1-zFootLand);
    else
      phZTemp = (1-phSingle) / (1-ph3FootPhase);
      fxf = 1;
      fzf = phZTemp^exp3FootPhase*zFootLand;
    end
  end
  return fxf, fzf;
end

entry();
