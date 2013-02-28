------------------------------------------------
-- Quadruped walk controller 
-- ZMP-based upper body movement
-- And IK based Arm and Leg movement
-- Walk, Trot, Pace, Gallop supported
-- 2013/2 SJ
------------------------------------------------

module(..., package.seeall);

require('Body')
require('Kinematics')
require('Config');
require('vector')
require('mcm')
require('unix')
require('util')

bodyHeight = Config.kneel.bodyHeight;
bodyTilt = Config.kneel.bodyTilt;
armX = Config.kneel.armX;
armY = Config.kneel.armY;
armZ = Config.kneel.armZ;
LArmRPY = Config.kneel.LArmRPY;
RArmRPY = Config.kneel.RArmRPY;
legX = Config.kneel.legX;
legY = Config.kneel.legY;
torsoX = Config.kneel.torsoX;

tStep = Config.kneel.tStep;
tZmp = Config.kneel.tZmp;
ph1Single = Config.kneel.ph1Single;
ph2Single = Config.kneel.ph2Single;
ph1Zmp,ph2Zmp=ph1Single,ph2Single;
stepHeight = Config.kneel.stepHeight;	

velLimitX = Config.kneel.velLimitX;
velLimitY = Config.kneel.velLimitY;
velLimitA = Config.kneel.velLimitA;

--Hardness parameters
hardnessSupport = Config.walk.hardnessSupport or 0.7;
hardnessSwing = Config.walk.hardnessSwing or 0.5;

----------------------------------------------------------
-- Walk state variables
----------------------------------------------------------

uTorso = vector.new({-torsoX, 0, 0});
uLLeg = vector.new({legX, legY, 0});
uRLeg = vector.new({legX, -legY, 0});
uLArm = vector.new({armX, armY, 0});
uRArm = vector.new({armX, -armY, 0});

uTorso1 = vector.new({-torsoX, 0, 0});
uLLeg1 = vector.new({legX, legY, 0});
uRLeg1 = vector.new({legX, -legY, 0});
uLArm1 = vector.new({armX, armY, 0});
uRArm1 = vector.new({armX, -armY, 0});

uTorso2 = vector.new({-torsoX, 0, 0});
uLLeg2 = vector.new({legX, legY, 0});
uRLeg2 = vector.new({legX, -legY, 0});
uLArm2 = vector.new({armX, armY, 0});
uRArm2 = vector.new({armX, -armY, 0});

pTorsoActual = vector.new({0, 0, bodyHeight, 0,bodyTilt,0});
pLLeg = vector.new({legX, legY, 0, 0,0,0});
pRLeg = vector.new({legX, -legY, 0, 0,0,0});
pLArm = vector.new({armX, armY, 0, LArmRPY[1],LArmRPY[2],LArmRPY[3]});
pRArm = vector.new({armX, -armY, 0, RArmRPY[1],RArmRPY[2],RArmRPY[3]});

velCurrent = vector.new({0, 0, 0});
velCommand = vector.new({0, 0, 0});

moving_legs={0,0,0,0};--LArm RArm LLeg RLeg

--ZMP exponential coefficients:
aXP, aXN, aYP, aYN = 0, 0, 0, 0;

---------------------------------------------------------

active = false;
started = false;
iStep0 = -1;
iStep = 0;
t0 = Body.get_time();
tLastStep = Body.get_time();
ph0=0;ph=0;

-------------------------------------------------------------

stopRequest = 2;
canWalkKick = 0; --Can we do walkkick with this walk code?
initial_step=2;
walk_style = 0; 

function entry()
  print ("Motion: Crawl entry")
  mcm.set_walk_bipedal(0);
end


function update()
  t = Body.get_time();

  if (not active) then
    zFoot = 0;     
    moving_legs={0,0,0,0};
    motion_limbs();
    return;    
  end
  if not started then
    started=true;
    tLastStep = Body.get_time();
  end

  ph = (t-tLastStep)/tStep;
  if ph>1 then
    iStep=iStep+1;
    ph=ph-math.floor(ph);
    tLastStep=tLastStep+tStep;
  end

  --New step
  if (iStep > iStep0) and (stopRequest ==1) then
    stopRequest = 0;
    active = false;
    return 'stop';
  end

  if (iStep > iStep0) then

    iStep0 = iStep;
    --Update initial positions
    uTorso1[1],uTorso1[2],uTorso1[3]=uTorso2[1],uTorso2[2],uTorso2[3];
    uLLeg1[1],uLLeg1[2],uLLeg1[3]=uLLeg2[1],uLLeg2[2],uLLeg2[3];
    uRLeg1[1],uRLeg1[2],uRLeg1[3]=uRLeg2[1],uRLeg2[2],uRLeg2[3];
    uLArm1[1],uLArm1[2],uLArm1[3]=uLArm2[1],uLArm2[2],uLArm2[3];
    uRArm1[1],uRArm1[2],uRArm1[3]=uRArm2[1],uRArm2[2],uRArm2[3];

    if walk_style==0 then
      do_walk_gait(iStep);
    elseif walk_style==1 then
      do_trot_gait(iStep);
    elseif walk_style==2 then
      do_pace_gait(iStep);
    else
      do_gallop_gait(iStep);
    end


--[[
  --com[3] = .5*(uLeft[3] + uRight[3]);
    print("New step: LA",uLArm1[1],uLArm2[1]);
    print("New step: RA",uRArm1[1],uRArm2[1]);
    print("New step: LL",uLLeg1[1],uLLeg2[1]);
    print("New step: RL",uRLeg1[1],uRLeg2[1]);
--]]

    uTorso2 = get_com(uLArm2,uRArm2,uLLeg2,uRLeg2);

    --Compute ZMP coefficients
    m1X = (uSupport[1]-uTorso1[1])/(tStep*ph1Zmp);
    m2X = (uTorso2[1]-uSupport[1])/(tStep*(1-ph2Zmp));
    m1Y = (uSupport[2]-uTorso1[2])/(tStep*ph1Zmp);
    m2Y = (uTorso2[2]-uSupport[2])/(tStep*(1-ph2Zmp));
    aXP, aXN = zmp_solve(uSupport[1], uTorso1[1], uTorso2[1],
    uTorso1[1], uTorso2[1]);
    aYP, aYN = zmp_solve(uSupport[2], uTorso1[2], uTorso2[2],
    uTorso1[2], uTorso2[2]);

  end
  xFoot,zFoot = foot_phase(ph);
  advance_limbs();
  advance_torso();
  motion_limbs();
end

function get_com(uA,uB,uC,uD) --get center of mass point
  if uD then --4 points
    local uCenterAB = util.se2_interpolate(0.5,uA,uB);
    local uCenterCD = util.se2_interpolate(0.5,uC,uD);
    local uCenterABCD = util.se2_interpolate(0.5,uCenterAB,uCenterCD);
    return uCenterABCD;
  else --3 points
    local midX = (uA[1]+uB[1]+uC[1])/3;
    local midY = (uA[2]+uB[2]+uC[2])/3;
    local midA = util.mod_angle((uA[3]+uB[3]+uC[3])/3);
    return vector.new({midX,midY,midA});
  end
end

function do_walk_gait(iStep)
  --Walk gait
  swingFoot = iStep % 4;
  if swingFoot == 0 then
    moving_legs={1,0,0,0};
    uLArm2=step_destination(uLArm1,vector.new({armX,armY,0}));
    uSupport = get_com(uRArm2,uLLeg2,uRLeg2);
  elseif swingFoot == 1 then
    moving_legs={0,0,0,1};
    uRLeg2=step_destination(uRLeg1,vector.new({legX,-legY,0}));
    uSupport = get_com(uLArm2,uRArm2,uLLeg2);
  elseif swingFoot == 2 then
    moving_legs={0,1,0,0};
    uRArm2=step_destination(uRArm1,vector.new({armX,-armY,0}));
    uSupport = get_com(uLArm2,uLLeg2,uRLeg2);
  else
    moving_legs={0,0,1,0};
    uLLeg2=step_destination(uLLeg1,vector.new({legX,legY,0}));
    uSupport = get_com(uLArm2,uRArm2,uRLeg2);
  end
--    uSupport = get_com(uLArm2,uRArm2,uLLeg2,uRLeg2);
end

function do_trot_gait(iStep)
  --Trot gait
  swingFoot = iStep % 2;
  if swingFoot == 0 then
    moving_legs={1,0,0,1};
    uLArm2=step_destination(uLArm1,vector.new({armX,armY,0}));
    uRLeg2=step_destination(uRLeg1,vector.new({legX,-legY,0}));
    uSupport = util.se2_interpolate(0.5,uRArm2,uLLeg2);
  elseif swingFoot == 1 then
    moving_legs={0,1,1,0};
    uRArm2=step_destination(uRArm1,vector.new({armX,-armY,0}));
    uLLeg2=step_destination(uLLeg1,vector.new({legX,legY,0}));
    uSupport = util.se2_interpolate(0.5,uLArm2,uRLeg2);
  end
end

function do_pace_gait(iStep)
  --Pace gait
  swingFoot = iStep % 2;
  if swingFoot == 0 then
    moving_legs={1,0,1,0};
    uLArm2=step_destination(uLArm1,vector.new({armX,armY,0}));
    uLLeg2=step_destination(uLLeg1,vector.new({legX,legY,0}));
    uSupport = util.se2_interpolate(0.5,uRArm2,uRLeg2);

  elseif swingFoot == 1 then
    moving_legs={0,1,0,1};
    uRArm2=step_destination(uRArm1,vector.new({armX,-armY,0}));
    uRLeg2=step_destination(uRLeg1,vector.new({legX,-legY,0}));
    uSupport = util.se2_interpolate(0.5,uLArm2,uLLeg2);

  end
end

function do_gallop_gait(iStep)
  --Gallop gait
  swingFoot = iStep % 2;
  if swingFoot == 0 then
    moving_legs={1,1,0,0};
    uLArm2=step_destination(uLArm1,vector.new({armX,armY,0}));
    uRArm2=step_destination(uRArm1,vector.new({armX,-armY,0}));
    uSupport = util.se2_interpolate(0.5,uLLeg2,uRLeg2);

  elseif swingFoot == 1 then
    moving_legs={0,0,1,1};
    uLLeg2=step_destination(uLLeg1,vector.new({legX,legY,0}));
    uRLeg2=step_destination(uRLeg1,vector.new({legX,-legY,0}));
    uSupport = util.se2_interpolate(0.5,uLArm2,uRArm2);

  end
end


function step_destination(uStart,uOffset)
  local uCenter = get_com(uLArm,uRArm,uLLeg,uRLeg);
  local u0 = util.pose_global({torsoX,0,0},uCenter);
  u1 = util.pose_global(velCurrent, u0);
  u2 = util.pose_global(0.5*velCurrent, u1);
  return util.pose_global(uOffset, u2);
end


function advance_limbs()
  uLLeg = util.se2_interpolate(xFoot,uLLeg1,uLLeg2);
  uRLeg = util.se2_interpolate(xFoot,uRLeg1,uRLeg2);
  uLArm = util.se2_interpolate(xFoot,uLArm1,uLArm2);
  uRArm = util.se2_interpolate(xFoot,uRArm1,uRArm2);
end

function advance_torso()
--  uTorso = get_com(uLArm,uRArm,uLLeg,uRLeg);
  uTorso = zmp_com(ph);
end

function motion_limbs()
  local uTorsoActual = util.pose_global({torsoX,0,0},uTorso);

  pTorsoActual[1],pTorsoActual[2],pTorsoActual[6] = 
	uTorsoActual[1],uTorsoActual[2],uTorsoActual[3];

  pLLeg[1],pLLeg[2],pLLeg[6]=uLLeg[1],uLLeg[2],uLLeg[3];
  pRLeg[1],pRLeg[2],pRLeg[6]=uRLeg[1],uRLeg[2],uRLeg[3];
  pLArm[1],pLArm[2],pLArm[6]=uLArm[1],uLArm[2],uLArm[3];
  pRArm[1],pRArm[2],pRArm[6]=uRArm[1],uRArm[2],uRArm[3];

  pLLeg[3]=0;pRLeg[3]=0;pLArm[3]=armZ;pRArm[3]=armZ;

  pLArm[3]=pLArm[3]+zFoot*stepHeight*moving_legs[1];
  pRArm[3]=pRArm[3]+zFoot*stepHeight*moving_legs[2];
  pLLeg[3]=pLLeg[3]+zFoot*stepHeight*moving_legs[3];
  pRLeg[3]=pRLeg[3]+zFoot*stepHeight*moving_legs[4];

  trTorso = Transform.transform6D(pTorsoActual);
  trLArm = Transform.transform6D(pLArm);
  trRArm = Transform.transform6D(pRArm);

  pLArmTorso = Transform.position6D(Transform.inv(trTorso)*trLArm);
  pRArmTorso = Transform.position6D(Transform.inv(trTorso)*trRArm);

  qLArm = Kinematics.inverse_l_arm(pLArmTorso);
  qRArm = Kinematics.inverse_r_arm(pRArmTorso);
  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorsoActual);


--Use wrist transform instead----------------------------------
  --TODO: arm NaN at kinematics 
  qLArm=Kinematics.inverse_l_wrist(pLArmTorso,5*math.pi/180);
  qRArm=Kinematics.inverse_r_wrist(pRArmTorso,-5*math.pi/180);
  qLArm[5]=-math.pi/2;
  qLArm[6]=-math.pi/2;
  qRArm[5]=math.pi/2;
  qRArm[6]=math.pi/2;
----------------------------------------------------------------



  Body.set_lleg_command(qLegs);
  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);

--  print(unpack(qLArm))

end

function exit()
end

function set_velocity(vx, vy, va)
  velCommand = vector.new({vx, vy, va});

  velCommand[1] = math.min(math.max(velCommand[1],velLimitX[1]),velLimitX[2]);
  velCommand[2] = math.min(math.max(velCommand[2],velLimitY[1]),velLimitY[2]);
  velCommand[3] = math.min(math.max(velCommand[3],velLimitA[1]),velLimitA[2]);

  velCurrent = velCommand; --TODO
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
  end
end

function stop()
  --Always stops with feet together (which helps kicking)
  stopRequest = math.max(1,stopRequest);
  --  stopRequest = 2; --Stop w/o feet together
end


function foot_phase(ph)
  -- Computes relative x,z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1);
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew));
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew));


--[[

  --Square pattern
  local trajA1=0.3;
  local trajA2=0.7;
  phFoot=.5*(1-math.cos(math.pi*phSingle));
  if phFoot<trajA1 then 
     xf=0;
     zf=phFoot/trajA1;
  elseif phFoot<trajA2 then
     xf = (phFoot-trajA1)/(trajA2-trajA1);
     zf = 1;
  else
     xf=1;
     zf=(1-phFoot)/(1-trajA2);
  end
--]]


  return xf, zf;
end




function get_odometry(u0)
  --TODO
  return vector.new({0,0,0});
end

function get_body_offset()
  --TODO
  return vector.new({0,0,0});
end

function stance_reset()
end
function upper_body_override_on() 
end
function upper_body_override() 
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

  --Linear speed turning
  com[3] = ph* (uLArm2[3]+uRArm2[3]+uLLeg2[3]+uRLeg2[3])/4 
              + (1-ph)* (uLArm1[3]+uRArm1[3]+uLLeg1[3]+uRLeg1[3])/4;

  return com;
end

