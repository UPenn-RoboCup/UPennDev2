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

--Stance parameters
bodyHeight = Config.walk.bodyHeight;
bodyTilt=Config.walk.bodyTilt or 0;
footX = Config.walk.footX or 0;
footY = Config.walk.footY;
supportX = Config.walk.supportX;
supportY = Config.walk.supportY;
qLArm=Config.walk.qLArm;
qRArm=Config.walk.qRArm;
qLArm0={qLArm[1],qLArm[2]};
qRArm0={qRArm[1],qRArm[2]};
hardnessSupport = Config.walk.hardnessSupport or 0.7;
hardnessSwing = Config.walk.hardnessSwing or 0.5;
hardnessArm = Config.walk.hardnessArm or 0.2;

--Gait parameters
tStep0 = Config.walk.tStep;
tStep = Config.walk.tStep;
tZmp = Config.walk.tZmp;
stepHeight = Config.walk.stepHeight;
ph1Single = Config.walk.phSingle[1];
ph2Single = Config.walk.phSingle[2];
ph1Zmp,ph2Zmp=ph1Single,ph2Single;

--Compensation parameters
hipRollCompensation = Config.walk.hipRollCompensation;
ankleMod = Config.walk.ankleMod or {0,0};

--Gyro stabilization parameters
ankleImuParamX = Config.walk.ankleImuParamX;
ankleImuParamY = Config.walk.ankleImuParamY;
kneeImuParamX = Config.walk.kneeImuParamX;
hipImuParamY = Config.walk.hipImuParamY;
armImuParamX = Config.walk.armImuParamX;
armImuParamY = Config.walk.armImuParamY;

--WalkKick parameters
walkKickVel = Config.walk.walkKickVel;
walkKickSupportMod = Config.walk.walkKickSupportMod;
walkKickHeightFactor = Config.walk.walkKickHeightFactor;
tStepWalkKick = Config.walk.tStepWalkKick or tStep;

--Sidekick parameters 
sideKickVel1 = Config.walk.sideKickVel1 or {0.04,0.04};
sideKickVel2 = Config.walk.sideKickVel2 or {0.09,0.05};
sideKickVel3 = Config.walk.sideKickVel3 or {0.09,-0.02};
sideKickSupportMod = Config.walk.sideKickSupportMod or {{0,0},{0,0}};
tStepSideKick = Config.walk.tStepSideKick or 0.70;

--Support bias parameters to reduce backlash-based instability
supportFront = Config.walk.supportFront or 0;
supportBack = Config.walk.supportBack or 0;
supportSide = Config.walk.supportSide or 0;

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
iStep0 = -1;
iStep = 0;
t0 = Body.get_time();
tLastStep = Body.get_time();

stopRequest = 2;
canWalkKick = 1; --Can we do walkkick with this walk code?
walkKickRequest = 0; 
walkKickType = 0;

initdone=false;
initial_step=2;

-------------------------------------------
--Boxing variables
-------------------------------------------

--Stance state --0: open, 1: left-front, 2: right-front 
stance,stance1=0,0; 

hardnessArm = 1;
velLimitX = {-.05, .05};
stanceLimitX={-0.08 , 0.08};

--Boxing arm pose
qLArm1=math.pi/180*vector.new({90,40,-160});
qRArm1=math.pi/180*vector.new({90,-40,-160});

--Straight arm pose
qLArm2=math.pi/180*vector.new({-20,40,0});
qRArm2=math.pi/180*vector.new({-20,-40,0});

qLArm2=math.pi/180*vector.new({-20,30,0});
qRArm2=math.pi/180*vector.new({-20,-30,0});

--Current arm pose
qLArm=math.pi/180*vector.new({90,40,-160});
qRArm=math.pi/180*vector.new({90,-40,-160});

qLArm0={qLArm[1],qLArm[2]};
qRArm0={qRArm[1],qRArm[2]};

--Standard offset 
uLRFootOffset = vector.new({0,footY,0});
uTorsoOffset = {-footX,0,0};

--Punch 
tPunch=0;
punchType=1;
punchTime={{0,0.12,0.12},{0.3,0.2,0.2}};

--punchTime={{0.4,0.4,0.4},{0.2,0.2,0.2}};


--For torso offset change
tStance0,tStance1 = 0,0;

----------------------------------------------------------
-- End initialization 
----------------------------------------------------------

function entry()
  print ("walk entry")
  --SJ: now we always assume that we start walking with feet together
  --Because joint readings are not always available with darwins

  uLeft = util.pose_global(vector.new({-supportX, footY, 0}),uTorso);
  uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso);

  uLeft1, uLeft2 = uLeft, uLeft;
  uRight1, uRight2 = uRight, uRight;
  uTorso1, uTorso2 = uTorso, uTorso;
  uSupport = uTorso;

  pLLeg = vector.new{uLeft[1], uLeft[2], 0, 0, 0, uLeft[3]};
  pRLeg = vector.new{uRight[1], uRight[2], 0, 0, 0, uRight[3]};
  pTorso = vector.new{uTorso[1], uTorso[2], bodyHeight, 0, bodyTilt, uTorso[3]};
   
  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, 0);
  Body.set_lleg_command(qLegs);

  --Place arms in appropriate position at sides
  Body.set_larm_command(qLArm);
  Body.set_larm_hardness(hardnessArm);
  Body.set_rarm_command(qRArm);
  Body.set_rarm_hardness(hardnessArm);

  walkKickRequest = 0;
  supportLeg=0;
end


function update()
  t = Body.get_time();

  if not active then
    if walkKickRequest==0 then 
      uTorso = step_torso(uLeft, uRight,0.5);
      pLLeg[3], pRLeg[3] = 0;
      motion_body();
      return; 
    elseif walkKickRequest==1 then
      tLastStep=t-tStep-0.001;
    end
  end

  --SJ: Variable tStep support for walkkick
  ph = (t-tLastStep)/tStep;
  if ph>1 then
    iStep=iStep+1;
    ph=ph-1;
    tLastStep=tLastStep+tStep;
    supportLeg=1-supportLeg;
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
    uLeft1 = uLeft2;
    uRight1 = uRight2;
    uTorso1 = uTorso2;
    check_stance();

    supportMod = {0,0}; --Support Point modulation for walkkick
    shiftFactor = 0.5; --How much should we shift final Torso pose?

    if walkKickRequest>0 then
      check_walkkick(); 
      check_side_walkkick(); 
      check_step_punch();
    --If stop signal sent, put two feet together
    elseif (stopRequest==1) then  --Final step
      stopRequest=2;
      velCurrent=vector.new({0,0,0});
      velCommand=vector.new({0,0,0});
      if supportLeg == 0 then-- Left support
	uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1);
      else        -- Right support
        uLeft2 = util.pose_global(2*uLRFootOffset, uRight1);
      end
    else --Normal walk, advance steps
      tStep=tStep0; 
      if supportLeg == 0 then-- Left support
        uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
      else  -- Right support
        uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
      end
    end
    uTorso2 = step_torso(uLeft2, uRight2,shiftFactor);

    if supportLeg == 0 then --LS
        uSupport = util.pose_global({supportX+supportMod[1], supportY+supportMod[2], 0}, uLeft);
        Body.set_lleg_hardness(hardnessSupport);
        Body.set_rleg_hardness(hardnessSwing);
    else --RS
        uSupport = util.pose_global({supportX+supportMod[1], -supportY-supportMod[2], 0}, uRight);
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
  end --End new step
  
  xFoot, zFoot = foot_phase(ph);  
  if initial_step>0 then zFoot=0;  end --Don't lift foot at initial step
  pLLeg[3], pRLeg[3] = 0;
  if supportLeg == 0 then    -- Left support
    if walkKickRequest == 4 and walkKickType>1 then --Side kick
      uRight = util.se3_interpolate(xFoot, uRight1, uRight15, uRight2);
    else
      uRight = util.se2_interpolate(xFoot, uRight1, uRight2);
    end
    pRLeg[3] = stepHeight*zFoot;
  else    -- Right support
    if walkKickRequest == 4 and walkKickType>1 then --side kick 
      uLeft = util.se3_interpolate(xFoot, uLeft1, uLeft15, uLeft2);
    else
      uLeft = util.se2_interpolate(xFoot, uLeft1, uLeft2);
    end
    pLLeg[3] = stepHeight*zFoot;
  end
  uTorso = zmp_com(ph);
  motion_body();
end

function motion_body()
  if t<tStance1 then -- Body stance changing
    torso_ph = (t-tStance0)/(tStance1-tStance0);
    uTorsoOffset = util.se2_interpolate(torso_ph,uTorsoOffset0,uTorsoOffsetTarget);
  else
    uTorsoOffset0 = uTorsoOffsetTarget;
  end    


  uTorsoActual = util.pose_global(uTorsoOffset,uTorso);
  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  pTorso[1], pTorso[2], pTorso[6] = uTorsoActual[1], uTorsoActual[2], uTorsoActual[3];
  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
  motion_legs(qLegs);
  motion_arms();
  Body.set_head_hardness(0.3);
  Body.set_head_command({-uTorsoOffset[3],0});
end

function check_stance()
  t=Body.get_time();
  uTorsoOffset0 = {uTorsoOffset[1],uTorsoOffset[2],uTorsoOffset[3]};
  if stance1==0 then    --Standard stance
    uLRFootOffset = vector.new({0,footY,0});
    uTorsoOffsetTarget = {-footX,0,0};
    tStance0=t;tStance1 = t+tStep;
  elseif stance1==1 then    --Left-front stance
    uLRFootOffset = vector.new({0.03,footY,0});
    uTorsoOffsetTarget = {-footX+0.01,0,-45*math.pi/180};
    tStance0=t;tStance1 = t+tStep;
  else    --Right-front stance
    uLRFootOffset = vector.new({-0.03,footY,0});
    uTorsoOffsetTarget = {-footX+0.01,0,45*math.pi/180};
    tStance0=t;tStance1 = t+tStep;
  end
  stance=stance1;
end

function check_walkkick()
    --Check walking kick phases
    if walkKickType>1 then return; end
    if walkKickRequest ==1 then --If support foot is right, skip 1st step
      if not active then supportLeg=walkKickType;end
      if supportLeg==walkKickType then walkKickRequest = 2;  end
    end
    if walkKickRequest == 1 then  -- Feet together
      if supportLeg == 0 then 
	uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1);
      else 
	uLeft2 = util.pose_global(2*uLRFootOffset, uRight1);
      end
    elseif walkKickRequest ==2 then     -- Support step forward
      if supportLeg == 0 then 
	uRight2 = util.pose_global({walkKickVel[1],-2*footY,0}, uLeft1);
        shiftFactor = 0.7; --shift final torso to right foot
      else 
	uLeft2 = util.pose_global({walkKickVel[1],2*footY,0}, uRight1); 
        shiftFactor = 0.3; --shift final torso to left foot
      end
      supportMod = walkKickSupportMod[1];
      tStep=tStepWalkKick;  --Slow down tStep for two kick step
    elseif walkKickRequest ==3 then       -- Kicking step forward
      if supportLeg == 0 then uRight2 = 
	util.pose_global({walkKickVel[2],-2*footY,0}, uLeft1);
      else uLeft2 = 
	util.pose_global({walkKickVel[2],2*footY,0}, uRight1);--RS
      end
      supportMod = walkKickSupportMod[2];
    elseif walkKickRequest == 4 then       -- Feet together
      if supportLeg == 0 then 
	uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1);
      else 
	uLeft2 = util.pose_global(2*uLRFootOffset, uRight1);
      end
      tStep=tStep0; 
    end
    walkKickRequest = (walkKickRequest+1)%6;
end


function check_side_walkkick()
    if walkKickType<2 or walkKickType>3 then return; end
    if walkKickRequest ==1 then --If support foot is right, skip 1st step
      if not active then supportLeg=walkKickType-2;end
      if supportLeg==walkKickType-2 then walkKickRequest = 2;    end
    end
    if walkKickRequest == 1 then -- Feet together
      if supportLeg == 0 then 
	uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1);
      else 
	uLeft2 = util.pose_global(2*uLRFootOffset, uRight1);
      end
    elseif walkKickRequest ==2 then  -- Support step side
      if supportLeg == 0 then 
	uRight2 = util.pose_global(
	  {sideKickVel1[1],-2*footY-sideKickVel1[2],0}, uLeft1);
        shiftFactor = 0.7; --shift final torso to right foot
      else 
	uLeft2 = util.pose_global(
	  {sideKickVel1[1],2*footY+sideKickVel1[2],0}, uRight1);
        shiftFactor = 0.3; --shift final torso to left foot
      end
      supportMod = sideKickSupportMod[1];
      tStep=tStepSideKick;  --Slow down tStep for two kick steps
    elseif walkKickRequest ==3 then    -- Kicking step side
      if supportLeg == 0 then 
	uRight15 = util.pose_global(
	  {sideKickVel2[1],-2*footY-sideKickVel2[2],0}, uLeft1);
	uRight2 = util.pose_global(
	  {sideKickVel3[1],-2*footY-sideKickVel3[2],0}, uLeft1);
      else 
	uLeft15 = util.pose_global(
	  {sideKickVel2[1],2*footY+sideKickVel2[2],0}, uRight1);
	uLeft2 = util.pose_global(
	  {sideKickVel3[1],2*footY+sideKickVel3[2],0}, uRight1);
      end
      supportMod = sideKickSupportMod[2];
    elseif walkKickRequest == 4 then      -- Feet together
      if supportLeg == 0 then uRight2 = util.pose_global({0,-2*footY,0}, uLeft1); 
      else uLeft2 = util.pose_global({0,2*footY,0}, uRight1); 
      end
      tStep=tStep0; 
    end
    walkKickRequest = (walkKickRequest+1)%6;
end


function check_step_punch()
--supportleg 0 : left support --stance 1: left front stance
    if stance==0 then return; end
    if walkKickType<4 then return; end

    if walkKickRequest==1 then
	if not active then supportLeg=stance-1;end
	if supportLeg+1==stance then walkKickRequest = 2; end
    end

    if walkKickRequest == 1 then  -- Feet at correct position
      if supportLeg == 0 then 
	uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1);
      else 
	uLeft2 = util.pose_global(2*uLRFootOffset, uRight1);
      end
    elseif walkKickRequest ==2 then     -- Support step forward
      switch_stance(3-stance); --change stance to opposite one
      check_stance();
      start_punch(2);
      if supportLeg == 0 then 
	uRight2 = util.pose_global(-2*uLRFootOffset, uLeft1);
--        shiftFactor = 0.7; --shift final torso to right foot
      else 
	uLeft2 = util.pose_global(2*uLRFootOffset, uRight1);
--        shiftFactor = 0.3; --shift final torso to left foot
      end
      supportMod = walkKickSupportMod[1];
      tStep=tStepWalkKick;  --Slow down tStep for two kick step
    end
    walkKickRequest = (walkKickRequest+1)%4;
end

function start_punch(ptype)
  if tPunch==0 then
    tPunch=t; 
    punchType=ptype;
  end
end


function motion_legs(qLegs)
  phComp = math.min(1, phSingle/.1, (1-phSingle)/.1);

  --Ankle stabilization using gyro feedback
  imuGyr = Body.get_sensor_imuGyrRPY();

  gyro_roll0=imuGyr[1];
  gyro_pitch0=imuGyr[2];
  --print("Gyro RPY", unpack(imuGyr))

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
  toeTipCompensation = 0;

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
    qLegs[2] = qLegs[2] + hipRollCompensation*phComp; --Hip roll compensation
  else
    qLegs[8] = qLegs[8]  + hipShift[2];    --Hip roll stabilization
    qLegs[10] = qLegs[10] + kneeShift;    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1];    --Ankle pitch stabilization
    qLegs[12] = qLegs[12] + ankleShift[2];    --Ankle roll stabilization

    qLegs[5] = qLegs[5]  + toeTipCompensation*phComp;--Lifting toetip
    qLegs[8] = qLegs[8] - hipRollCompensation*phComp;--Hip roll compensation
  end

--[[
  local spread=(uLeft[3]-uRight[3])/2;
  qLegs[5] = qLegs[5] + Config.walk.anklePitchComp[1]*math.cos(spread);
  qLegs[11] = qLegs[11] + Config.walk.anklePitchComp[2]*math.cos(spread);
--]]

  Body.set_lleg_command(qLegs);
end

--[[
function motion_arms()
  qLArm[1],qLArm[2]=qLArm0[1]+armShift[1],qLArm0[2]+armShift[2];
  qRArm[1],qRArm[2]=qRArm0[1]+armShift[1],qRArm0[2]+armShift[2];
  qLArm[2]=math.max(8*math.pi/180,qLArm[2])
  qRArm[2]=math.min(-8*math.pi/180,qRArm[2]);

  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);
end
--]]

function motion_arms()

  pTime=punchTime[punchType];
  arm_ph=0;
  if tPunch==0 then arm_ph=0; 
  elseif t-tPunch<pTime[1] then
    arm_ph=0;
  elseif t-tPunch<pTime[1]+pTime[2] then
    arm_ph = (t-tPunch-pTime[1])/pTime[2];
--    arm_ph=1;
  elseif t-tPunch<pTime[1]+pTime[2]+pTime[3] then
    arm_ph = 1-(t-tPunch-pTime[1]-pTime[2])/pTime[3];
  else 
    tPunch=0;arm_ph=0;
  end

  qLArm[1],qLArm[2],qLArm[3]= qLArm1[1],qLArm1[2],qLArm1[3];
  qRArm[1],qRArm[2],qRArm[3]= qRArm1[1],qRArm1[2],qRArm1[3];

  if stance==1 then --Left straight
    qLArm=util.se2_interpolate(arm_ph,qLArm1,qLArm2);
  elseif stance==2 then
    qRArm=util.se2_interpolate(arm_ph,qRArm1,qRArm2);
  end

  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);
end

function exit()
end

function step_left_destination(vel, uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0);
  local u2 = util.pose_global(.5*vel, u1);
  local uLeftPredict = util.pose_global(uLRFootOffset, u2);
  local uLeftRight = util.pose_relative(uLeftPredict, uRight);
  -- Do not pidgeon toe, cross feet:
  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2]);
  uLeftRight[2] = math.min(math.max(uLeftRight[2], stanceLimitY[1]), stanceLimitY[2]);
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2]);

  return util.pose_global(uLeftRight, uRight);
end

function step_right_destination(vel, uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0);
  local u2 = util.pose_global(.5*vel, u1);
  local uRightPredict = util.pose_global(-uLRFootOffset, u2);
  local uRightLeft = util.pose_relative(uRightPredict, uLeft);
  -- Do not pidgeon toe, cross feet:

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2]);
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -stanceLimitY[1]);
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1]);

  return util.pose_global(uRightLeft, uLeft);
end

function step_torso(uLeft, uRight,shiftFactor)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeft);
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight);
  return util.se2_interpolate(shiftFactor, uLeftSupport, uRightSupport);
end

function set_velocity(vx, vy, vz)
  magFactor = 1;
  velCommand[1]=vx*magFactor;
  velCommand[2]=vy*magFactor;
  velCommand[3]=vz;

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
    iStep0 = -1;
    t0 = Body.get_time();
    tLastStep = Body.get_time();
    initdone=false;
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
    walkKickType = 0; --Start with left support
  end
end

function doWalkKickRight()
 if walkKickRequest==0 then
    walkKickRequest = 1; 
    walkKickType = 1; --Start with right support
  end
end

function doSideKickLeft()
 if walkKickRequest==0 then
    walkKickRequest = 1; 
    walkKickType = 2; 
  end
end

function doSideKickRight()
 if walkKickRequest==0 then
    walkKickRequest = 1; 
    walkKickType = 3; 
  end
end

function doPunch(punchtype)
  if punchtype==1 then --light punch
    start_punch(1);
  elseif punchtype==2 then --stepping straight
    if walkKickRequest==0 then
      walkKickRequest = 1; 
      walkKickType = 4; 
    end
  end

end

function stance_reset() --standup/sitdown/falldown handling
  uTorsoOffset = {-footX,0,0};stance=0;  
  check_stance();
end

function switch_stance(stance)
  stance1=stance;
end

--dummy function for NSL kick, depreciated
function zero_velocity()
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
  com[3] = .5*(uLeft[3] + uRight[3]);
  return com;
end

function foot_phase(ph)
  -- Computes relative x,z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1);
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew));
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew));

  --hack: vertical takeoff and landing
--  factor1 = 0.2;
  factor1 = 0;
  factor2 = 0;
  phSingleSkew2 = math.max(
	math.min(1,
	(phSingleSkew-factor1)/(1-factor1-factor2)
	 ), 0);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew2));

  --Check for walkkick step
  if walkKickRequest == 4 then 
    zf = zf * walkKickHeightFactor; --Increase step height
    if walkKickType <2 then --Different trajectory for Front walkkick
      local kickN = 1.5; 
      if phSingle<0.5 then xf=kickN*phSingle;
      else xf = (1-kickN)*(2*phSingle-1) + kickN;
      end
    end
  end

  return xf, zf;
end

entry();
