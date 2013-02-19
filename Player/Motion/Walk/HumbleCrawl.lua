--THOR-OP specific (FOR 6DOF ARM)

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
footX = Config.kneel.footX;
footY = Config.kneel.footY;

tStep = Config.kneel.tStep or 0.5;
stepHeight = Config.kneel.stepHeight or 0.06;	

velLimitX = Config.kneel.velLimitX or {-.06, .08};
velLimitY = Config.kneel.velLimitY or {-.06, .06};
velLimitA = Config.kneel.velLimitA or {-.4, .4};

--Hardness parameters
hardnessSupport = Config.walk.hardnessSupport or 0.7;
hardnessSwing = Config.walk.hardnessSwing or 0.5;

----------------------------------------------------------
-- Walk state variables
----------------------------------------------------------

uTorso = vector.new({0, 0, 0});
uLLeg = vector.new({footX, footY, 0});
uRLeg = vector.new({footX, -footY, 0});
uLArm = vector.new({armX, armY, 0});
uRArm = vector.new({armX, -armY, 0});

uTorso1 = vector.new({0, 0, 0});
uLLeg1 = vector.new({footX, footY, 0});
uRLeg1 = vector.new({footX, -footY, 0});
uLArm1 = vector.new({armX, armY, 0});
uRArm1 = vector.new({armX, -armY, 0});

uTorso2 = vector.new({0, 0, 0});
uLLeg2 = vector.new({footX, footY, 0});
uRLeg2 = vector.new({footX, -footY, 0});
uLArm2 = vector.new({armX, armY, 0});
uRArm2 = vector.new({armX, -armY, 0});




pTorso = vector.new({0, 0, bodyHeight, 0,bodyTilt,0});
pLLeg = vector.new({footX, footY, 0, 0,0,0});
pRLeg = vector.new({footX, -footY, 0, 0,0,0});
pLArm = vector.new({armX, armY, 0, LArmRPY[1],LArmRPY[2],LArmRPY[3]});
pRArm = vector.new({armX, -armY, 0, RArmRPY[1],RArmRPY[2],RArmRPY[3]});

velCurrent = vector.new({0, 0, 0});
velCommand = vector.new({0, 0, 0});

moving_legs={0,0,0,0};--LArm RArm LLeg RLeg


---------------------------------------------------------

active = true;
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


function entry()
  print ("Motion: Crawl entry")
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
  if (iStep > iStep0) then
    iStep0 = iStep;
    --Update initial positions
    uTorso1[1],uTorso1[2],uTorso1[3]=uTorso2[1],uTorso2[2],uTorso2[3];
    uLLeg1[1],uLLeg1[2],uLLeg1[3]=uLLeg2[1],uLLeg2[2],uLLeg2[3];
    uRLeg1[1],uRLeg1[2],uRLeg1[3]=uRLeg2[1],uRLeg2[2],uRLeg2[3];
    uLArm1[1],uLArm1[2],uLArm1[3]=uLArm2[1],uLArm2[2],uLArm2[3];
    uRArm1[1],uRArm1[2],uRArm1[3]=uRArm2[1],uRArm2[2],uRArm2[3];


    --Crawl gait
    swingFoot = iStep % 4;
    if swingFoot == 0 then
      moving_legs={1,0,0,0};
      uLArm2=step_destination(uLArm1,vector.new({armX,armY,0}));
    elseif swingFoot == 1 then
      moving_legs={0,0,0,1};
      uRLeg2=step_destination(uRLeg1,vector.new({footX,-footY,0}));
    elseif swingFoot == 2 then
      moving_legs={0,1,0,0};
      uRArm2=step_destination(uRArm1,vector.new({armX,-armY,0}));
    else
      moving_legs={0,0,1,0};
      uLLeg2=step_destination(uLLeg1,vector.new({footX,footY,0}));
    end

--[[
    --Trot gait
    swingFoot = iStep % 2;
    if swingFoot == 0 then
      moving_legs={1,0,0,1};
      uLArm2=step_destination(uLArm1,vector.new({armX,armY,0}));
      uRLeg2=step_destination(uRLeg1,vector.new({footX,-footY,0}));
    elseif swingFoot == 1 then
      moving_legs={0,1,1,0};
      uRArm2=step_destination(uRArm1,vector.new({armX,-armY,0}));
      uLLeg2=step_destination(uLLeg1,vector.new({footX,footY,0}));
    end
--]]


--[[
    print("New step: LA",uLArm1[1],uLArm2[1]);
    print("New step: RA",uRArm1[1],uRArm2[1]);
    print("New step: LL",uLLeg1[1],uLLeg2[1]);
    print("New step: RL",uRLeg1[1],uRLeg2[1]);
--]]


  end
  advance_limbs();
  motion_limbs();
end


function step_destination(uStart,uOffset)
  --TODO
  local uCenterArm = util.se2_interpolate(0.5,uLArm,uRArm);
  local uCenterLeg = util.se2_interpolate(0.5,uLLeg,uRLeg);
--  local u0 = util.se2_interpolate(-footX/(armX-footX),
--	uCenterArm,uCenterLeg);

  local u0 = util.se2_interpolate(armX/(armX-footX),
	uCenterArm,uCenterLeg);

  u1 = util.pose_global(velCurrent, u0);
  u2 = util.pose_global(0.5*velCurrent, u1);
  
  return util.pose_global(uOffset, u2);

end


function advance_limbs()
  xFoot,zFoot = foot_phase(ph);

  uLLeg = util.se2_interpolate(xFoot,uLLeg1,uLLeg2);
  uRLeg = util.se2_interpolate(xFoot,uRLeg1,uRLeg2);
  uLArm = util.se2_interpolate(xFoot,uLArm1,uLArm2);
  uRArm = util.se2_interpolate(xFoot,uRArm1,uRArm2);

--  uTorso = util.se2_interpolate(-footX/(armX-footX),
--	uCenterArm,uCenterLeg);

  uCenterArm = util.se2_interpolate(0.5,uLArm,uRArm);
  uCenterLeg = util.se2_interpolate(0.5,uLLeg,uRLeg);

  uTorso = util.se2_interpolate(armX/(armX-footX),
	uCenterArm,uCenterLeg);


end

function motion_limbs()

  pTorso[1],pTorso[2],pTorso[6]=uTorso[1],uTorso[2],uTorso[3];

  pLLeg[1],pLLeg[2],pLLeg[6]=uLLeg[1],uLLeg[2],uLLeg[3];
  pRLeg[1],pRLeg[2],pRLeg[6]=uRLeg[1],uRLeg[2],uRLeg[3];

  pLArm[1],pLArm[2],pLArm[6]=uLArm[1],uLArm[2],uLArm[3];
  pRArm[1],pRArm[2],pRArm[6]=uRArm[1],uRArm[2],uRArm[3];

  pLLeg[3]=0;pRLeg[3]=0;
  pLArm[3]=armZ;pRArm[3]=armZ;

  pLArm[3]=pLArm[3]+zFoot*stepHeight*moving_legs[1];
  pRArm[3]=pRArm[3]+zFoot*stepHeight*moving_legs[2];
  pLLeg[3]=pLLeg[3]+zFoot*stepHeight*moving_legs[3];
  pRLeg[3]=pRLeg[3]+zFoot*stepHeight*moving_legs[4];


  trTorso = Transform.transform6D(pTorso);
  trLArm = Transform.transform6D(pLArm);
  trRArm = Transform.transform6D(pRArm);

  pLArmTorso = Transform.position6D(Transform.inv(trTorso)*trLArm);
  pRArmTorso = Transform.position6D(Transform.inv(trTorso)*trRArm);

  qLArm = Kinematics.inverse_l_arm(pLArmTorso);
  qRArm = Kinematics.inverse_r_arm(pRArmTorso);
  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso);

  Body.set_lleg_command(qLegs);
  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);
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

ph1Single = 0.15;
ph2Single = 0.85;

  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1);
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew));
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew));
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

