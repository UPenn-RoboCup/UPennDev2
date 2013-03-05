require('Platform')
require('vector')
require('Config')
require('Transform')
require('Kinematics')
require('Motion_state')

----------------------------------------------------------------------
-- ZMP Walking
----------------------------------------------------------------------

-- Setup 
----------------------------------------------------------------------
walk = Motion_state.new('walk')
walk:set_joint_access(0, 'all')
walk:set_joint_access(1, 'lowerbody')
local dcm = walk.dcm

-- define default parameters
walk.parameters = {
}

-- load config parameters
walk:load_parameters()


-- Gait Parameters
----------------------------------------------
-- Stance and velocity limit values
----------------------------------------------
local stanceLimitX={-0.60,0.60};
local stanceLimitY={0.16,0.60};
local stanceLimitA={-10*math.pi/180,30*math.pi/180};
local velLimitX={-.20,.40};
local velLimitY={-.15,.15};
local velLimitA={-.3,.3};
local velDelta={0.10,0.10,0.15}

----------------------------------------------
-- Stance parameters
---------------------------------------------
local bodyHeight = 0.75;
local bodyTilt=4*math.pi/180;
local footX= 0.01;
local footY = 0.09;
local supportX = -0.03;
local supportY = -0.01;
local hardnessSupport = 1;
local hardnessSwing = 1;

---------------------------------------------
-- Gait parameters
---------------------------------------------
local tStep = 0.6;
local tZmp = 0.25;
local stepHeight = 0.04;
local ph1Single,ph2Single=0.1,0.9;
local ph1Zmp,ph2Zmp=0.1,0.9;

--Compensation parameters
local hipRollCompensation = 1.5*math.pi/180;
local ankleMod = vector.new({-1,0})/0.12 * 10*math.pi/180;

--Gyro stabilization parameters
local gyroFactor = 0.273*math.pi/180 * 300 / 1024*0.2; --based on deg/s unit
local ankleImuParamX={1,0.75*gyroFactor, 2*math.pi/180, 10*math.pi/180};
local kneeImuParamX={1,1.5*gyroFactor, 2*math.pi/180, 10*math.pi/180};
local ankleImuParamY={1,1*gyroFactor, 2*math.pi/180, 10*math.pi/180};
local hipImuParamY={1,1*gyroFactor, 2*math.pi/180, 10*math.pi/180};

----------------------------------------------------------
-- Walk state variables
----------------------------------------------------------
local uTorso = vector.new({supportX, 0, 0});
local uLeft = vector.new({0, footY, 0});
local uRight = vector.new({0, -footY, 0});

local pLLeg = vector.new({0, footY, 0, 0,0,0});
local pRLeg = vector.new({0, -footY, 0, 0,0,0});
local pTorso = vector.new({supportX, 0, bodyHeight, 0,bodyTilt,0});

local velCurrent = vector.new({0, 0, 0});
local velCommand = vector.new({0, 0, 0});
local velDiff = vector.new({0, 0, 0});

--ZMP exponential coefficients:
local aXP, aXN, aYP, aYN = 0, 0, 0, 0;

--Gyro stabilization variables
local ankleShift = vector.new({0, 0});
local kneeShift = 0;
local hipShift = vector.new({0,0});

local iStep0 = -1;
local iStep = 0;
local t0 = Platform.get_time();
local tLastStep = Platform.get_time();

local active = false;
local stopRequest=0;
local initdone=false;
local tInit=0;
local initial_step=2;

----------------------------------------------------------
-- End initialization 
----------------------------------------------------------

local t0 = Platform.get_time()
local q0 = dcm:get_joint_position_sensor('legs')
local qStance = vector.copy(q0)

----------------------------------------------------------
-- Utilities
----------------------------------------------------------

local function mod_angle(a)
  -- Reduce angle to [-pi, pi)
  a = a % (2*math.pi);
  if (a >= math.pi) then
    a = a - 2*math.pi;
  end
  return a;
end

local function se2_interpolate(t, u1, u2)
  -- helps smooth out the motions using a weighted average
  return vector.new{u1[1]+t*(u2[1]-u1[1]),
                    u1[2]+t*(u2[2]-u1[2]),
                    u1[3]+t*mod_angle(u2[3]-u1[3])};
end

local function procFunc(a,deadband,maxvalue)
  --Piecewise linear function for IMU feedback
  if a>0 then
        b=math.min( math.max(0,math.abs(a)-deadband), maxvalue);
  else
        b=-math.min( math.max(0,math.abs(a)-deadband), maxvalue);
  end
  return b;
end

local function pose_global(pRelative, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  return vector.new{pose[1] + ca*pRelative[1] - sa*pRelative[2],
                    pose[2] + sa*pRelative[1] + ca*pRelative[2],
                    pose[3] + pRelative[3]};
end

local function pose_relative(pGlobal, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  local px = pGlobal[1]-pose[1];
  local py = pGlobal[2]-pose[2];
  local pa = pGlobal[3]-pose[3];
  return vector.new{ca*px + sa*py, -sa*px + ca*py, mod_angle(pa)};
end

local function step_left_destination(vel, uLeft, uRight)
  local u0 = se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = pose_global(vel, u0);
  local u2 = pose_global(.5*vel, u1);
  local uLeftPredict = pose_global({0, footY, 0}, u2);
  local uLeftRight = pose_relative(uLeftPredict, uRight);
  -- Do not pidgeon toe, cross feet:

  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2]);
  uLeftRight[2] = math.min(math.max(uLeftRight[2], stanceLimitY[1]), stanceLimitY[2]);
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2]);

  return pose_global(uLeftRight, uRight);
end

local function step_right_destination(vel, uLeft, uRight)
  local u0 = se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = pose_global(vel, u0);
  local u2 = pose_global(.5*vel, u1);
  local uRightPredict = pose_global({0, -footY, 0}, u2);
  local uRightLeft = pose_relative(uRightPredict, uLeft);
  -- Do not pidgeon toe, cross feet:

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2]);
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -stanceLimitY[1]);
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1]);

  return pose_global(uRightLeft, uLeft);
end

local function step_torso(uLeft, uRight,shiftFactor)
  local u0 = se2_interpolate(.5, uLeft, uRight);
  local uLeftSupport = pose_global({supportX, supportY, 0}, uLeft);
  local uRightSupport = pose_global({supportX, -supportY, 0}, uRight);
  return se2_interpolate(shiftFactor, uLeftSupport, uRightSupport);
end

local function update_velocity()
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

local function zmp_solve(zs, z1, z2, x1, x2)
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
local function zmp_com(ph)
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

local function foot_phase(ph)
  -- Computes relative x,z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1);
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew));
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew));
  return xf, zf;
end

local function motion_legs(qLegs)
  phComp = math.min(1, phSingle/.1, (1-phSingle)/.1);

--[[
  --Ankle stabilization using gyro feedback
  imuGyr = Platform.get_sensor_imuGyrRPY();
  gyro_roll0=imuGyr[1];
  gyro_pitch0=imuGyr[2];
--]]

  gyro_roll0,gyro_pitch0=0,0; --Disable feedback for now

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

  ankleShiftX=procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4]);
  ankleShiftY=procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4]);
  kneeShiftX=procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4]);
  hipShiftY=procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4]);

  ankleShift[1]=ankleShift[1]+ankleImuParamX[1]*(ankleShiftX-ankleShift[1]);
  ankleShift[2]=ankleShift[2]+ankleImuParamY[1]*(ankleShiftY-ankleShift[2]);
  kneeShift=kneeShift+kneeImuParamX[1]*(kneeShiftX-kneeShift);
  hipShift[2]=hipShift[2]+hipImuParamY[1]*(hipShiftY-hipShift[2]);

  toeTipCompensation = 0;

  if not active then --Double support, standing still
    qLegs[4] = qLegs[4] + kneeShift;    --Knee pitch stabilization
    qLegs[5] = qLegs[5]  + ankleShift[1];    --Ankle pitch stabilization

    qLegs[10] = qLegs[10] + kneeShift;    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1];    --Ankle pitch stabilization

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

  -- write joint angles to shared memory 
  -- Platform.set_lleg_command(qLegs);
  dcm:set_joint_position(qLegs, 'legs')

end

local function update_still()
  uTorso = step_torso(uLeft, uRight,0.5);
  uTorsoActual = pose_global(vector.new({-footX,0,0}),uTorso);
  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  pLLeg[3], pRLeg[3] = 0;
  pTorso[1], pTorso[2], pTorso[6] = uTorsoActual[1], uTorsoActual[2], uTorsoActual[3];

  local trLLeg = Transform.pose(pLLeg);
  local trRleg = Transform.pose(pRLeg);
  local trTorso = Transform.pose(pTorso);
  qLegs = Kinematics.inverse_pos_legs(trLLeg, trRLeg, trTorso, supportLeg);
  motion_legs(qLegs);
end

local function init()
  --Lower body height for walking

  t = Platform.get_time();
  init_time = 3.0;
  init_ph = (t-tInit) / init_time;

  bodyHeight0 = 0.9;
  bodyTilt0 = 0;

  bodyHeightInit = init_ph*bodyHeight + (1-init_ph)*bodyHeight0;
  bodyTiltInit = init_ph*bodyTilt + (1-init_ph)*bodyTilt0;

  pLLeg = vector.new{uLeft[1], uLeft[2], 0, 0, 0, uLeft[3]};
  pRLeg = vector.new{uRight[1], uRight[2], 0, 0, 0, uRight[3]};
  pTorso = vector.new{uTorso[1], uTorso[2], 
	bodyHeightInit, 0, bodyTiltInit, uTorso[3]};
   
  local trLLeg = Transform.pose(pLLeg);
  local trRleg = Transform.pose(pRLeg);
  local trTorso = Transform.pose(pTorso);
  qLegs = Kinematics.inverse_pos_legs(trLLeg, trRLeg, trTorso);

--  Platform.set_lleg_command(qLegs);
  dcm:set_joint_position(qLegs, 'legs')

  if init_ph>1 then
    initdone=true;
    tLastStep=t;
  end
end

----------------------------------------------------------------------
-- Interface
----------------------------------------------------------------------

function walk:start()
  stopRequest = 0;
  if (not active) then
    active = true;
    iStep0 = -1;
    t0 = Platform.get_time();
    tLastStep = Platform.get_time();
    initial_step=2;
  end
end

function walk:stop()
  stopRequest = math.max(1,stopRequest);
end

function walk:is_active()
  -- return true is gait is active
  return active
end

function walk:set_velocity(vx, vy, va)
  --Filter the commanded speed
  magFactor = 1;
  velCommand[1] = vx*magFactor;
  velCommand[2] = vy*magFactor;
  velCommand[3] = va;

  velCommand[1] = math.min(math.max(velCommand[1],velLimitX[1]),velLimitX[2]);
  velCommand[2] = math.min(math.max(velCommand[2],velLimitY[1]),velLimitY[2]);
  velCommand[3] = math.min(math.max(velCommand[3],velLimitA[1]),velLimitA[2]);
end

function walk:get_velocity()
  return vector.copy(velCommand) --vector.copy(velCurrent)
end

function walk:exit()
end

function walk:entry()
  tInit = Platform.get_time();

  uLeft = pose_global(vector.new({-supportX, footY, 0}),uTorso);
  uRight = pose_global(vector.new({-supportX, -footY, 0}),uTorso);

  uLeft1, uLeft2 = uLeft, uLeft;
  uRight1, uRight2 = uRight, uRight;
  uTorso1, uTorso2 = uTorso, uTorso;

  uSupport = uTorso;

  local q0 = dcm:get_joint_position_sensor('legs')
  dcm:set_joint_force(0, 'legs')
  dcm:set_joint_position(q0, 'legs')
  dcm:set_joint_velocity(0, 'legs')
  dcm:set_joint_stiffness(1, 'legs')
  dcm:set_joint_damping(0, 'legs')
end

function walk:update()
  if (not initdone) then
     init();
     return;
  end

  if (not active) then 
    update_still();
    return; 
  end

  t = Platform.get_time();

  ph = (t-tLastStep)/tStep;
  if ph>1 then
    iStep=iStep+1;
    ph=ph-1;
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

    if (stopRequest==1) then  --Final step
      stopRequest=2;
      velCurrent=vector.new({0,0,0});
      velCommand=vector.new({0,0,0});
      if supportLeg == 0 then        -- Left support
        uRight2 = pose_global({0,-2*footY,0}, uLeft1);
      else        -- Right support
        uLeft2 = pose_global({0,2*footY,0}, uRight1);
      end
    else --Normal walk, advance steps
      if supportLeg == 0 then-- Left support
        uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
      else  -- Right support
        uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
      end
    end

    uTorso2 = step_torso(uLeft2, uRight2,0.5);

    if supportLeg == 0 then --LS
	local uLeftTorso = pose_relative(uLeft1,uTorso1);
        uSupport = pose_global({supportX, supportY, 0},uLeft);
    else --RS
	local uRightTorso = pose_relative(uRight1,uTorso1);
        uSupport = pose_global({supportX, -supportY, 0}, uRight);
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
    uRight = se2_interpolate(xFoot, uRight1, uRight2);
    pRLeg[3] = stepHeight*zFoot;
  else    -- Right support
    uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
    pLLeg[3] = stepHeight*zFoot;
  end

  uTorso = zmp_com(ph);
  uTorsoActual = pose_global(vector.new({-footX,0,0}),uTorso);
  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  pTorso[1], pTorso[2], pTorso[6] = uTorsoActual[1], uTorsoActual[2], uTorsoActual[3];

  local trLLeg = Transform.pose(pLLeg);
  local trRleg = Transform.pose(pRLeg);
  local trTorso = Transform.pose(pTorso);
  qLegs = Kinematics.inverse_pos_legs(trLLeg, trRLeg, trTorso, supportLeg);
  motion_legs(qLegs);
end

return walk
