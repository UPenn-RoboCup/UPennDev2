module(..., package.seeall);

------------------------------------------------
-- This function uses ZMP preview algorithm
-- To make robot a number of pre-defined steps
-- 2013/2 SJ
------------------------------------------------

require('Body')
require('Kinematics')
require('Config');
require('vector')
require('mcm')
require('unix')
require('util')

local matrix = require('matrix_zmp')

--Stance parameters
bodyHeight1 = Config.zmpstep.bodyHeight;
bodyHeight0 = Config.walk.bodyHeight;
bodyHeight = Config.walk.bodyHeight;

bodyTilt=Config.zmpstep.bodyTilt or 0;

footX = Config.walk.footX or 0;
footY = Config.walk.footY;

qLArm=Config.walk.qLArm;
qRArm=Config.walk.qRArm;
qLArm0={qLArm[1],qLArm[2]};
qRArm0={qRArm[1],qRArm[2]};
hardnessSupport = Config.walk.hardnessSupport or 0.7;
hardnessSwing = Config.walk.hardnessSwing or 0.5;
hardnessArm = Config.walk.hardnessArm or 0.2;

--Gait parameters
tZmp = Config.zmpstep.tZmp;
supportX = Config.zmpstep.supportX;
supportY = Config.zmpstep.supportY;
stepHeight = Config.zmpstep.stepHeight;
ph1Single = Config.zmpstep.phSingle[1];
ph2Single = Config.zmpstep.phSingle[2];

--Compensation parameters
hipRollCompensation = Config.zmpstep.hipRollCompensation;

--Gyro stabilization parameters
ankleImuParamX = Config.walk.ankleImuParamX;
ankleImuParamY = Config.walk.ankleImuParamY;
kneeImuParamX = Config.walk.kneeImuParamX;
hipImuParamY = Config.walk.hipImuParamY;
armImuParamX = Config.walk.armImuParamX;
armImuParamY = Config.walk.armImuParamY;

----------------------------------------------------------
-- Walk state variables
----------------------------------------------------------

uTorso = vector.new({0, 0, 0});
uLeft = vector.new({-supportX, footY, 0});
uRight = vector.new({-supportX, -footY, 0});

--Future positions (for preview control)
uTorsoF = vector.new({0, 0, 0});
uLeftF = vector.new({-supportX, footY, 0});
uRightF = vector.new({-supportX, -footY, 0});
uSupportF = vector.new({0,0,0});

supportLegF = 2; --DS

pLLeg = vector.new({-supportX, footY, 0, 0,0,0});
pRLeg = vector.new({-supportX, -footY, 0, 0,0,0});
pTorso = vector.new({supportX, 0, bodyHeight, 0,bodyTilt,0});

velCurrent = vector.new({0, 0, 0});
velCommand = vector.new({0, 0, 0});

--ZMP exponential coefficients:
aXP, aXN, aYP, aYN = 0, 0, 0, 0;

--Gyro stabilization variables
ankleShift = vector.new({0, 0});
kneeShift = 0;
hipShift = vector.new({0,0});
armShift = vector.new({0, 0});

active = false;
t0 = Body.get_time();
t1 = Body.get_time();

--ZMP preview parameters
timeStep = 0.015; --15ms
tPreview = 1.60; --preview interval, 1600ms
nPreview = tPreview / timeStep; 
r_q = 10^-6; --balacing parameter for optimization

--Zmp preview variables
x={}
zmpx={};zmpy={};
uLeftTargets={};
uRightTargets={};
supportLegs={};
phs={};

------------------------------------------------------
--Step queue information

gapX = 0.75;
gapY = 0.0;

gapX = 0.25;
gapY = -0.30;

gapX = 0.80;
gapY = -0.40;

step_queue= {
   {
    {-supportX, footY, 0}, --uLeft
    {-supportX+gapX, -footY+gapY, 0}, --uRight
    0, -- Left support
    0.8, --duration
   },
   {
    {-supportX+gapX, footY+gapY, 0}, --uLeft
    {-supportX+gapX, -footY+gapY, 0}, --uRight
    1, -- Right Support
    0.8, --duration
   },
   {
    {-supportX+gapX, footY+gapY, 0}, --uLeft
    {-supportX+gapX, -footY+gapY, 0}, --uRight
    2, -- Double support
    0.8, --duration
   },
};

gap0={0,0}
gap1={0.50,0}
gap2={0.75,0.20}

gap3={1.05,0.60};
gap4={1.30,0.50};

gap5={1.95,0.20};
gap6={2.25,0.10};

gap7={2.95,0.10};
gap7={3.0,0.10};





step_queue= {
--Initial wait 
   {
    {-supportX+gap0[1], footY+gap0[2], 0}, --uLeft
    {-supportX+gap0[1], -footY+gap0[2], 0}, --uRight
    2, -- Right support
    1, --duration
   },

--First cross
   {
    {-supportX+gap1[1], footY+gap1[2], 0}, --uLeft
    {-supportX+gap0[1], -footY+gap0[2], 0}, --uRight
    1, -- Right support
    0.8, --duration
   },
   {
    {-supportX+gap1[1], footY+gap1[2], 0}, --uLeft
    {-supportX+gap1[1], -footY+gap1[2], 0}, --uRight
    0, -- LS
    0.8, --duration
   },
--One step forward

   {
    {-supportX+gap2[1], footY+gap2[2], 0}, --uLeft
    {-supportX+gap1[1], -footY+gap1[2], 0}, --uRight
    1, -- RS
    0.8, --duration
   },
   {
    {-supportX+gap2[1], footY+gap2[2], 0}, --uLeft
    {-supportX+gap2[1], -footY+gap2[2], 0}, --uRight
    0, -- LS
    0.8, --duration
   },
   {
    {-supportX+gap2[1], footY+gap2[2], 0}, --uLeft
    {-supportX+gap2[1], -footY+gap2[2], 0}, --uRight
    2, -- LS
    2.5, --duration
   },


--Second cross
   {
    {-supportX+gap3[1], footY+gap3[2], 0}, --uLeft
    {-supportX+gap2[1], -footY+gap2[2], 0}, --uRight
    1, -- RS
    0.8, --duration
   },
   {
    {-supportX+gap3[1], footY+gap3[2], 0}, --uLeft
    {-supportX+gap3[1], -footY+gap3[2], 0}, --uRight
    0, -- LS 
    0.8, --duration
   },
   {
    {-supportX+gap3[1], footY+gap3[2], 0}, --uLeft
    {-supportX+gap3[1], -footY+gap3[2], 0}, --uRight
    2, -- DS 
    2.5, --duration
--    0.8, --duration
   },
--Step forward
   {
    {-supportX+gap4[1], footY+gap4[2], 0}, --uLeft
    {-supportX+gap3[1], -footY+gap3[2], 0}, --uRight
    1, -- RS
    0.8, --duration
   },
   {
    {-supportX+gap4[1], footY+gap4[2], 0}, --uLeft
    {-supportX+gap4[1], -footY+gap4[2], 0}, --uRight
    0, -- LS 
    0.8, --duration
   },
   {
    {-supportX+gap4[1], footY+gap4[2], 0}, --uLeft
    {-supportX+gap4[1], -footY+gap4[2], 0}, --uRight
    2, -- DS 
    0.8, --duration
   },


--Third cross

   {
    {-supportX+gap4[1], footY+gap4[2], 0}, --uLeft
    {-supportX+gap5[1], -footY+gap5[2], 0}, --uRight
    0, -- LS 
    0.8, --duration
   },
   {
    {-supportX+gap5[1], footY+gap5[2], 0}, --uLeft
    {-supportX+gap5[1], -footY+gap5[2], 0}, --uRight
    1, -- RS
    0.8, --duration
   },
   {
    {-supportX+gap5[1], footY+gap5[2], 0}, --uLeft
    {-supportX+gap5[1], -footY+gap5[2], 0}, --uRight
    2, -- RS
    1.5, --duration
   },

--step forward
   {
    {-supportX+gap5[1], footY+gap5[2], 0}, --uLeft
    {-supportX+gap6[1], -footY+gap6[2], 0}, --uRight
    0, -- LS
    0.8, --duration
   },
   {
    {-supportX+gap6[1], footY+gap6[2], 0}, --uLeft
    {-supportX+gap6[1], -footY+gap6[2], 0}, --uRight
    1, -- RS 
    0.8, --duration
   },
   {
    {-supportX+gap6[1], footY+gap6[2], 0}, --uLeft
    {-supportX+gap6[1], -footY+gap6[2], 0}, --uRight
    2, -- DS 
    1.5, --duration
   },

--Final crossing

   {
    {-supportX+gap6[1], footY+gap6[2], 0}, --uLeft
    {-supportX+gap7[1], -footY+gap7[2], 0}, --uRight
    0, -- LS 
    0.8, --duration
   },
   {
    {-supportX+gap7[1], footY+gap7[2], 0}, --uLeft
    {-supportX+gap7[1], -footY+gap7[2], 0}, --uRight
    1, -- RS 
    0.8, --duration
   },
   {
    {-supportX+gap7[1], footY+gap7[2], 0}, --uLeft
    {-supportX+gap7[1], -footY+gap7[2], 0}, --uRight
    2, --DS
    0.8, --duration
   },
};















step_queue_count = 0;
step_queue_t0 = 0;

----------------------------------------------------------
-- End initialization 
----------------------------------------------------------

function precompute()
  px={};pu0={};pu={};
  for i=1, nPreview do
    px[i]={1, i*timeStep, i*i*timeStep*timeStep/2 - tZmp*tZmp};
    pu0[i]=(1+3*(i-1)+3*(i-1)^2)/6 *timeStep^3 - timeStep*tZmp*tZmp;
    pu[i]={};
    for j=1, nPreview do pu[i][j]=0; end
    for j0=1,i do
      j = i+1-j0;
      pu[i][j]=pu0[i-j+1];
    end
  end
  param_pu = matrix:new(pu)
  param_px = matrix:new(px)
  param_pu_trans = matrix.transpose(param_pu);
  param_a=matrix {{1,timeStep,timeStep^2/2},{0,1,timeStep},{0,0,1}};
  param_b=matrix.transpose({{timeStep^3/6, timeStep^2/2, timeStep,timeStep}}) ;
  param_eye = matrix:new(nPreview,"I");
  param_k=-matrix.invert(
        (param_pu_trans * param_pu) + (r_q*param_eye)
        )* param_pu_trans ;
  k1={};
  k1[1]={};
  for i=1,nPreview do k1[1][i]=param_k[1][i];end
  param_k1 = matrix:new(k1);
  param_k1_px = param_k1 * param_px;
end

function stance_reset()
  --Quick hack: reset uTorso 
  --TODO: integrated odometry handling at Motion.lua
  uTorso = {0,0,0};

  --Clear current position variables
  uLeft = util.pose_global(vector.new({-supportX, footY, 0}),uTorso);
  uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso);
  uLeft1, uLeft2 = uLeft, uLeft;
  uRight1, uRight2 = uRight, uRight;
  uSupport = uTorso;

  --Clear future trajectory variable
  uLeftF, uLeftF = uLeft, uLeft;
  uRightF, uRightF = uRight, uRight;
  supportLegF = 0;

  --Clear trajectory queue
  x=matrix:new{{uTorso[1],uTorso[2]},{0,0},{0,0}};
  for i=1,nPreview do
     zmpx[i]=uTorso[1];
     zmpy[i]=uTorso[2]; 
     supportLegs[i]=2; --double support
     uLeftTargets[i]={uLeft[1],uLeft[2],uLeft[3]};
     uRightTargets[i]={uRight[1],uRight[2],uRight[3]};
     phs[i]= 0;
  end

  --reset step queue count
  step_queue_t0 = Body.get_time();
  step_queue_count = 0;

  bodyHeight = bodyHeight0; --Start from walking body height
end

function entry()
  print ("walk entry")
  --SJ: now we always assume that we start walking with feet together
  --Because joint readings are not always available with darwins
  stance_reset();

  --Place arms in appropriate position at sides
  Body.set_larm_command(qLArm);
  Body.set_larm_hardness(hardnessArm);
  Body.set_rarm_command(qRArm);
  Body.set_rarm_hardness(hardnessArm);

  active = true; --Automatically start stepping
end

function update_zmp_array()
  t = Body.get_time();
  local new_step = false;
  if step_queue_count== 0 then
    step_queue_t0 = t;
    new_step = true;
  elseif step_queue_count == #step_queue then
    supportLegF = 3; --This means END state
  elseif t>step_queue_t0 + step_queue[step_queue_count][4] then
    step_queue_t0 = step_queue_t0 + step_queue[step_queue_count][4];
    new_step = true;
  end

  if new_step then
    step_queue_count = step_queue_count + 1;
    supportLegF = step_queue[step_queue_count][3];
    uLeftF = step_queue[step_queue_count][1];
    uRightF = step_queue[step_queue_count][2];

    if supportLegF == 0 then-- Left support
      uSupportF = util.pose_global({supportX, supportY, 0}, uLeftF);
    elseif supportLegF == 1 then  -- Right support
      uSupportF = util.pose_global({supportX, -supportY, 0}, uRightF);
    else --Double support
      uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeftF);
      uRightSupport = util.pose_global({supportX, -supportY, 0}, uRightF);
      uSupportF = util.se2_interpolate(0.5,uLeftSupport,uRightSupport);
    end
  end

  if supportLegF ~=3 then 
    phF = (t-step_queue_t0)/step_queue[step_queue_count][4];
  else
    phF = 0;
  end

  table.remove(zmpx,1);
  table.remove(zmpy,1);
  table.remove(uLeftTargets,1);
  table.remove(uRightTargets,1);
  table.remove(supportLegs,1);
  table.remove(phs,1);

  table.insert(zmpx, uSupportF[1]);
  table.insert(zmpy, uSupportF[2]);
  table.insert(uLeftTargets, {uLeftF[1],uLeftF[2],uLeftF[3]});
  table.insert(uRightTargets,{uRightF[1],uRightF[2],uRightF[3]});
  table.insert(supportLegs, supportLegF);
  table.insert(phs, phF);
end

function update()
  if (not active) then 
    t0 = Body.get_time();
    t1 = Body.get_time();
    return; 
  end

  t = Body.get_time();
  update_zmp_array();

  dpLimit = .1; 

  --Lower the bodyHeight to prepare stepping
  local bodyHeightDiff = bodyHeight-bodyHeight1;

  bodyHeight = bodyHeight - math.min(
	bodyHeightDiff, dpLimit*timeStep);

  --Stop when stopping sequence is done
  if (supportLegs[1]==3) then --End state reached
    print("STOPPED")
    active = false;
    return "done";
  end

  if supportLeg ~= supportLegs[1] then --New step
    t1=t;
    uLeft = uLeft1;
    uRight = uRight1;
    uLeft0 = uLeft1;
    uRight0 = uRight1;
    supportLeg = supportLegs[1];
    uLeft1=uLeftTargets[1];
    uRight1=uRightTargets[1];
    if supportLeg == 0 then --LS
        Body.set_lleg_hardness(hardnessSupport);
        Body.set_rleg_hardness(hardnessSwing);
    elseif supportLeg==1 then --RS
        Body.set_lleg_hardness(hardnessSwing);
        Body.set_rleg_hardness(hardnessSupport);
    end
  end

  ph = phs[1];
  xFoot, zFoot = foot_phase(ph);  
  if not active then zFoot = 0; end

  if supportLeg == 0 then    -- Left support
    uRight = util.se2_interpolate(xFoot, uRight0, uRight1);
    pLLeg[3]=0;
    pRLeg[3] = stepHeight*zFoot;

  elseif supportLeg==1 then    -- Right support
    uLeft = util.se2_interpolate(xFoot, uLeft0, uLeft1);
    pLLeg[3] = stepHeight*zFoot;
    pRLeg[3]=0;
  end

  -- Get state feedback
  imuAngle = Body.get_sensor_imuAngle();
  imuGyr = Body.get_sensor_imuGyrRPY();
  imuRoll = imuAngle[1];
  imuPitch = imuAngle[2]-bodyTilt;
  gyro_roll=imuGyr[1];
  gyro_pitch=imuGyr[2];

  x_err0 = {math.sin(imuPitch)*bodyHeight,
    	   -math.sin(imuRoll)*bodyHeight};

  x_err={x_err0[1]*math.cos(uTorso[3])-x_err0[2]*math.sin(uTorso[3]),
	x_err0[2]*math.cos(uTorso[3])+x_err0[1]*math.sin(uTorso[3])}

  x_err[1]=math.min(0.06,math.max(-0.06,x_err[1]));
  x_err[2]=math.min(0.06,math.max(-0.06,x_err[2]));
--  print("x_err:",unpack(x_err))


  threshold = 0.02;
  --Deadzone filtering
  if x_err[1]>threshold then x_err[1]=x_err[1]-threshold;
  elseif x_err[1]<-threshold then x_err[1]=x_err[1]+threshold;
  else x_err[1]=0;
  end
  if x_err[2]>threshold then x_err[2]=x_err[2]-threshold;
  elseif x_err[2]<-threshold then x_err[2]=x_err[2]+threshold;
  else x_err[2]=0;
  end

  feedback_gain1 = 1;
  feedback_gain1 = 0;

  x_closed=x[1][1]+x_err[1]*feedback_gain1;
  y_closed=x[1][2]+x_err[2]*feedback_gain1;

  --  Update state variable
  --  u = param_k1_px * x - param_k1* zmparray; --Control output
  --  x = param_a * x + param_b * u;

  ux = param_k1_px[1][1] * x_closed+
	param_k1_px[1][2] * x[2][1]+
	param_k1_px[1][3] * x[3][1];

  uy =  param_k1_px[1][1] * y_closed+
	param_k1_px[1][2] * x[2][2]+
	param_k1_px[1][3] * x[3][2];

  for i=1,nPreview do
    ux = ux - param_k1[1][i]*zmpx[i];
    uy = uy - param_k1[1][i]*zmpy[i];
  end

  feedback_gain2 = 0;

  x[1][1]=x[1][1]+x_err[1]*feedback_gain2;
  x[1][2]=x[1][2]+x_err[2]*feedback_gain2;
  x= param_a*x + param_b * matrix:new({{ux,uy}});

  uTorso=vector.new({x[1][1],x[1][2],(uLeft[3]+uRight[3])/2 });
  uTorsoActual = util.pose_global(vector.new({-footX,0,0}),uTorso);

  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  pTorso[1], pTorso[2], pTorso[6] = uTorsoActual[1], uTorsoActual[2], uTorsoActual[3];
  pTorso[3] = bodyHeight;

  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
  motion_legs(qLegs);
  motion_arms();
end


function motion_legs(qLegs)
  phComp = math.min(1, phSingle/.1, (1-phSingle)/.1);

  --Ankle stabilization using gyro feedback
  imuGyr = Body.get_sensor_imuGyrRPY();

  gyro_roll=imuGyr[1];
  gyro_pitch=imuGyr[2];

  --Hack: No gyro-based ankle strategy here
  gyro_roll, gyro_pitch =0,0;

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

  if supportLeg == 0 then  -- Left support
    qLegs[2] = qLegs[2] + hipShift[2];    --Hip roll stabilization
    qLegs[4] = qLegs[4] + kneeShift;    --Knee pitch stabilization
    qLegs[5] = qLegs[5]  + ankleShift[1];    --Ankle pitch stabilization
    qLegs[6] = qLegs[6] + ankleShift[2];    --Ankle roll stabilization
    qLegs[11] = qLegs[11]  + toeTipCompensation;
    qLegs[2] = qLegs[2] + hipRollCompensation*phComp; --Hip roll compensation
  elseif supportLeg==1 then --Right support
    qLegs[8] = qLegs[8]  + hipShift[2];    --Hip roll stabilization
    qLegs[10] = qLegs[10] + kneeShift;    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1];    --Ankle pitch stabilization
    qLegs[12] = qLegs[12] + ankleShift[2];    --Ankle roll stabilization

    --Lifting toetip
    qLegs[5] = qLegs[5]  + toeTipCompensation;
    qLegs[8] = qLegs[8] - hipRollCompensation*phComp;--Hip roll compensation
  else --double support
    qLegs[4] = qLegs[4] + kneeShift;    --Knee pitch stabilization
    qLegs[5] = qLegs[5]  + ankleShift[1];    --Ankle pitch stabilization
    qLegs[10] = qLegs[10] + kneeShift;    --Knee pitch stabilization
    qLegs[11] = qLegs[11]  + ankleShift[1];    --Ankle pitch stabilization
  end

  Body.set_lleg_command(qLegs);
end

function motion_arms()
  qLArm[1],qLArm[2]=qLArm0[1]+armShift[1],qLArm0[2]+armShift[2];
  qRArm[1],qRArm[2]=qRArm0[1]+armShift[1],qRArm0[2]+armShift[2];
  qLArm[2]=math.max(8*math.pi/180,qLArm[2])
  qRArm[2]=math.min(-8*math.pi/180,qRArm[2]);

  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);
end

function exit()
end

function start()
  if (not active) then
    active = true;
    t0 = Body.get_time();
    stance_reset();
  end
end

function stop()
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

function foot_phase(ph)
  -- Computes relative x,z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1);
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew));
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew));

  factor1 = 0;
  factor2 = 0;
  phSingleSkew2 = math.max(
	math.min(1,
	(phSingleSkew-factor1)/(1-factor1-factor2)
	 ), 0);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew2));

  return xf, zf;
end

precompute();


function set_velocity()
end
