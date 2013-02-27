module(..., package.seeall);

require('Body')
require('Kinematics')
require('Config');
require('vector')
require('mcm')
require('unix')
require('util')

local matrix = require('matrix')

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
iStep0 = -1;
iStep = 0;
t0 = Body.get_time();
t1 = Body.get_time();

stopRequest = 2;
canWalkKick = 0; --Can we do walkkick with this walk code?

initdone=false;

--ZMP preview parameters
timeStep = 0.015; --15ms
nPreview = 80; --preview interval, 80*10ms = 0.8sec

nPreview = 150; --preview interval, 150*15ms 


r_q = 10^-6; --balacing parameter for optimization

--Zmp preview variables
x={}
zmpx={};zmpy={};
uLeftTargets={};
uRightTargets={};
supportLegs={};


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
  uLeft = util.pose_global(vector.new({-supportX, footY, 0}),uTorso);
  uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso);
  uLeft1, uLeft2 = uLeft, uLeft;
  uRight1, uRight2 = uRight, uRight;
  uLeftF1, uLeftF2 = uLeft, uLeft;
  uRightF1, uRightF2 = uRight, uRight;
  uTorso1, uTorso2 = uTorso, uTorso;
  uSupport = uTorso;
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

  x=matrix:new{{uTorso[1],uTorso[2]},{0,0},{0,0}};
  for i=1,nPreview do
     zmpx[i]=uTorso[1];
     zmpy[i]=uTorso[2]; 
     supportLegs[i]=2; --double support
     uLeftTargets[i]={uLeft[1],uLeft[2],uLeft[3]};
     uRightTargets[i]={uRight[1],uRight[2],uRight[3]};
  end
  if Config.sit_disable>0 then
    active = false;
  end
end

function update_zmp_array()
  t = Body.get_time();
  local iStepF, phF = math.modf((t-t0)/tStep);

  if (iStepF>iStepF0) then
    if stopRequest>0 or not active then --stop
      supportLegF = 2; --Double support end

      uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeftF1);
      uRightSupport = util.pose_global({supportX, -supportY, 0}, uRightF1);
      uSupportF = util.se2_interpolate(0.5,uLeftSupport,uRightSupport);

      uLeftF1 = uLeftF2;
      uRightF1 = uRightF2;
    else
      update_velocity();
      uLeftF1 = uLeftF2;
      uRightF1 = uRightF2;
      supportLegF = 1-supportLegF;

      if supportLegF == 0 then-- Left support
        uRightF2 = step_right_destination(velCurrent, uLeftF1, uRightF1);
        uSupportF = util.pose_global({supportX, supportY, 0}, uLeftF1);
      else  -- Right support
        uLeftF2 = step_left_destination(velCurrent, uLeftF1, uRightF1);
        uSupportF = util.pose_global({supportX, -supportY, 0}, uRightF1);
      end
    end
  end

  iStepF0=iStepF;

  table.remove(zmpx,1);
  table.remove(zmpy,1);
  table.remove(uLeftTargets,1);
  table.remove(uRightTargets,1);
  table.remove(supportLegs,1);
  table.insert(zmpx, uSupportF[1]);
  table.insert(zmpy, uSupportF[2]);
  table.insert(uLeftTargets, {uLeftF2[1],uLeftF2[2],uLeftF2[3]});
  table.insert(uRightTargets,{uRightF2[1],uRightF2[2],uRightF2[3]});
  table.insert(supportLegs, supportLegF);
end


function update_zmp_array2()
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

  table.remove(zmpx,1);
  table.remove(zmpy,1);
  table.remove(uLeftTargets,1);
  table.remove(uRightTargets,1);
  table.remove(supportLegs,1);

  table.insert(zmpx, uSupportF[1]);
  table.insert(zmpy, uSupportF[2]);
  table.insert(uLeftTargets, {uLeftF[1],uLeftF[2],uLeftF[3]});
  table.insert(uRightTargets,{uRightF[1],uRightF[2],uRightF[3]});
  table.insert(supportLegs, supportLegF);
end





function update()
  if (not active) then 
    t0 = Body.get_time();
    t1 = Body.get_time();
    return; 
  end

  if (stopRequest==2) then
    return 'done';
  end

  t = Body.get_time();
--  update_zmp_array();
  update_zmp_array2();

--print(supportLegs[1],zmpy[1]);

  --Stop when stopping sequence is done
--  if (supportLegs[1]==2) and(stopRequest>0) then
  if (supportLegs[1]==3) then --End state reached
      uTorsoTarget = util.se2_interpolate(0.5,uLeft,uRight);
      dist = math.sqrt((uTorsoTarget[1]-uTorso[1])^2 + 
	       (uTorsoTarget[2]-uTorso[2])^2);

      if dist < 0.005 then
        stopRequest = 0;
        active = false;
--        return "done";
      end
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


  iStep, ph = math.modf((t-t1)/tStep);
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

  --State feedback
  imuAngle = Body.get_sensor_imuAngle();
  imuRoll = imuAngle[1];
  imuPitch = imuAngle[2]-bodyTilt;

  imuGyr = Body.get_sensor_imuGyrRPY();
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

--Update state variable
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

--[[
print("zmpy:")
print(unpack(zmpy))
print("Torso y:",x[1][2]);
--]]

  uTorso=vector.new({x[1][1],x[1][2],(uLeft[3]+uRight[3])/2 });
  uTorsoActual = util.pose_global(vector.new({-footX,0,0}),uTorso);

  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  pTorso[1], pTorso[2], pTorso[6] = uTorsoActual[1], uTorsoActual[2], uTorsoActual[3];

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

function step_left_destination(vel, uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0);
  local u2 = util.pose_global(.5*vel, u1);
  local uLeftPredict = util.pose_global({0, footY, 0}, u2);
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
  local uRightPredict = util.pose_global({0, -footY, 0}, u2);
  local uRightLeft = util.pose_relative(uRightPredict, uLeft);
  -- Do not pidgeon toe, cross feet:

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2]);
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -stanceLimitY[1]);
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1]);

  return util.pose_global(uRightLeft, uLeft);
end

function step_torso(uLeft, uRight)
  local u0 = util.se2_interpolate(.5, uLeft, uRight);
  local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeft);
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight);
  return util.se2_interpolate(.5, uLeftSupport, uRightSupport);
end

function set_velocity(vx, vy, vz)
  --Filter the commanded speed
--[[
  vz= math.min(math.max(vz,velLimitA[1]),velLimitA[2]);
  local stepMag=math.sqrt(vx^2+vy^2);
  local magFactor=math.min(0.10,stepMag)/(stepMag+0.000001);
--]]

  magFactor = 1;
  velCommand[1]=vx*magFactor;
  velCommand[2]=vy*magFactor;
  velCommand[3]=vz;
end

function update_velocity()
  local velDiff={};
  velDiff[1]= math.min(math.max(velCommand[1]-velCurrent[1],
	-velDelta[1]),velDelta[1]);
  velDiff[2]= math.min(math.max(velCommand[2]-velCurrent[2],
	-velDelta[2]),velDelta[2]);
  velDiff[3]= math.min(math.max(velCommand[3]-velCurrent[3],
	-velDelta[3]),velDelta[3]);

  velCurrent[1] = math.min(math.max(velCurrent[1]+velDiff[1],
	velLimitX[1]),velLimitX[2]);
  velCurrent[2] = math.min(math.max(velCurrent[2]+velDiff[2],
	velLimitY[1]),velLimitY[2]);
  velCurrent[3] = math.min(math.max(velCurrent[3]+velDiff[3],
	velLimitA[1]),velLimitA[2]);
end

function get_velocity()
  return velCurrent;
end

function start()
  stopRequest = 0;
  if (not active) then
    active = true;
    supportLegF = 0;

    iStep0 = -1;
    iStepF0 = -1;
    t0 = Body.get_time();
    initdone=false;

    step_queue_t0 = Body.get_time();

  end
end

function stop()
  stopRequest = math.max(1,stopRequest);
--  stopRequest = 2;
end

function stopAlign()
  stop()
end

--dummy function for NSL kick
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


--Dummy functions

function set_push_recovery(ankle,hip,step)
end

function start_push_recovery(fx,fy)
end
