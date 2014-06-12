--Walk code for NSL 2011
--Made by SJ, Dec 2010
--DO NOT DISTRIBUTE!

module(..., package.seeall);

require('Body')
require('Kinematics')
require('Config');
require('vector')
require('util')
step=require('NSLStep')

mod_angle = util.mod_angle

--Can we do walkkick with this walk code?
canWalkKick = 1; 

-- Walk Parameters
footX = Config.walk.footX+Config.walk.footXComp;
footY = Config.walk.footY+Config.walk.footYComp;
supportX = Config.walk.supportX;
supportY = Config.walk.supportY+Config.walk.supportYComp;
tStep = Config.walk.tStep;

--Foot position parameters
stanceLimitX=Config.walk.stanceLimitX;
stanceLimitY=Config.walk.stanceLimitY;
stanceLimitA=Config.walk.stanceLimitA;

--Velocity limit parameters
velLimitX=Config.walk.velLimitX;
velLimitY=Config.walk.velLimitY;
velLimitA=Config.walk.velLimitA;

--Acceleration limit parameters
velDelta=Config.walk.velDelta;

--Kick recovery 
starting_foot=0; 

--Arm and leg hardness
qLArm=Config.walk.qLArm;
qRArm=Config.walk.qRArm;

hardnessArm=Config.walk.hardnessArm;
hardnessLeg=Config.walk.hardnessLeg;

--Foot and torso poses
uLeft = vector.new({0, footY, 0});
uRight = vector.new({0, -footY, 0});
uBody = vector.new({supportX, 0, 0});
uTorso = vector.new({-footX+supportX, 0, 0});
uZmp = vector.new({supportX, 0, 0});

uLeft2 = vector.new({0, footY, 0});
uRight2 = vector.new({0, -footY, 0});
uBody2 = vector.new({supportX, 0, 0});

--Walk state variables
velAlpha=0.5;
velCurrent = vector.new({0, 0, 0});
velCommand = vector.new({0, 0, 0});

t0 = 0;
iStep0 = -1;
enable=true;
active = true;

stopRequest = false;

ph=0;
ph0=0;
walkphase=0;

just_started=false;
supportLeg=0;
initial_step=0;
has_ball=0;

tWalkKick=0;
tWalkKickDelay=1.0;

function entry()
  print("Walk entry")
  velCurrent = vector.new({0, 0, 0});
  velCommand = vector.new({0, 0, 0});
  uLeft1 = uLeft;uLeft2 = uLeft;
  uRight1 = uRight;uRight2 = uRight;
  uBody1 = uBody;uBody2 = uBody;
  uSupport = uBody;
  uZmp=uBody;
  uTorso=pose_global(vector.new({-footX,0,0}),uBody);
  iStep0 = -1;
  t0 = Body.get_time();
  tWalkKick=0;
  active=enable;

  just_started=true;
  Body.set_larm_hardness(hardnessArm);
  Body.set_rarm_hardness(hardnessArm);
  Body.set_lleg_hardness(hardnessLeg);
  Body.set_rleg_hardness(hardnessLeg);
  step.reset(uLeft,uBody,uRight,t0,starting_foot);
end

function exit()
end

function update()
  t = Body.get_time();
  step.has_ball=has_ball;
  if (not active) then 
     step.update(t);
     uLeft,uRight,uBody,uTorso,uZmp=step.getPos();
     ph,ph0,walkphase=step.ph,step.ph0,step.walkphase;  
     return;
  end

  if not step.isactive(t) then
    update_velocity();

    if (stopRequest) then
      print("Walk stopped")
      stopRequest = false;
      active = false;
      tWalkKick=0;
      return "stop";
    end
    uLeft1 = step.uLeft2;
    uRight1 = step.uRight2;
    uBody1 = step.uBody2;
    if  just_started==false then
      supportLeg=1 - step.supportLeg;
    end
    just_started=false;
 
    local supportXMod=0;
    local supportYMod=0;

    if initial_step==1 then
	velCurrent=vector.new({0,0,0});
        initial_step=2;
    elseif initial_step==2 then
	velCurrent=vector.new({0,0,0});
        initial_step=0;
    end

--Support bias for fast walking forward
    if velCurrent[1]>0.08 then
	supportXMod=Config.walk.supportXfactor0*1.3;
    elseif velCurrent[1]>0.06 then
	supportXMod=Config.walk.supportXfactor0;
    end

--Support bias for walking backward
    if velCurrent[1]<0 then
	supportXMod=velCurrent[1]*Config.walk.supportXfactor1+Config.walk.supportXfactor2;
    end

--Support bias for sidestepping
    supportYMod= velCurrent[2]*Config.walk.supportYfactor1; 
    supportXMod=supportXMod+math.abs(velCurrent[2])*Config.walk.supportXfactor3;


    if supportLeg == 0 then  -- Left support
    	uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
	local uLeftBody=pose_relative(uLeft1,uBody1);
        local uBodyModed=pose_global(vector.new({supportXMod,supportYMod,0}),uBody1);
	local uLeftModed=pose_global(uLeftBody,uBodyModed)
	uSupport=pose_global({supportX,supportY,0},uLeftModed);
    else  -- Right support
     	uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
	local uRightBody=pose_relative(uRight1,uBody1);
        local uBodyModed=pose_global(vector.new({supportXMod,supportYMod,0}),uBody1);
	local uRightModed=pose_global(uRightBody,uBodyModed)
	uSupport=pose_global({supportX,-supportY,0},uRightModed);
    end

    uBody2 = step_body(uLeft2, uRight2);

    if initial_step==2 then
	--special initial step
	print("Initial step")
--    	step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,2);
    	step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,0.40 ,2);
    else
    	step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);
    end

--print(string.format("walk enque: L0 %.4f B0 %.4f R0 %.4f -> L1 %.4f B1 %.4f R1 %.4f",
--	uLeft1[1],uBody1[1],uRight1[1],uLeft2[1],uBody2[1],uRight2[1]));

  end
  step.update(t);  
  uLeft,uRight,uBody,uTorso,uZmp=step.getPos();
  ph,ph0,walkphase=step.ph,step.ph0,step.walkphase;  
end


function step_left_destination(vel, uLeft, uRight)
  local u0 = se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = pose_global(vel, u0);
  local u2 = pose_global(.5*vel, u1);
  local uLeftPredict = pose_global({0, footY, 0}, u2);
  local uLeftRight = pose_relative(uLeftPredict, uRight);

  --Prevent robot from stepping on heel edge
  local limitY=stanceLimitY[1];
  if uLeftRight[3]>0.3 then 
     limitY=limitY+0.015;
  end

  -- Do not pidgeon toe, cross feet:
  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2]);
  uLeftRight[2] = math.min(math.max(uLeftRight[2], limitY), stanceLimitY[2]);
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2]);
  
  local L1= pose_global(uLeftRight, uRight);

--print("Step Left: R0 Vel R1",uLeft[1],vel[1],L1[1]);

  return pose_global(uLeftRight, uRight);
end

function step_right_destination(vel, uLeft, uRight)
  local u0 = se2_interpolate(.5, uLeft, uRight);
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = pose_global(vel, u0);
  local u2 = pose_global(.5*vel, u1);
  local uRightPredict = pose_global({0, -footY, 0}, u2);
  local uRightLeft = pose_relative(uRightPredict, uLeft);

  --Prevent robot from stepping on heel edge
  local limitY=stanceLimitY[1];
  if uRightLeft[3]<-0.3 then 
     limitY=limitY+0.015;
  end

  -- Do not pidgeon toe, cross feet:
  uRightLeft[1] = math.min(math.max(uRightLeft[1], -stanceLimitX[2]), -stanceLimitX[1]);
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -limitY);--OP specific
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1]);

  local R1=  pose_global(uRightLeft, uLeft);

--print("Step Right: L0 Vel L1",uRight[1],vel[1],R1[1]);

  return pose_global(uRightLeft, uLeft);
end

function step_body(uL, uR)
  local u0 = se2_interpolate(.5, uL, uR);
  local uLeftSupport = pose_global({supportX, supportY, 0}, uL);
  local uRightSupport = pose_global({supportX, -supportY, 0}, uR);
  local uB=se2_interpolate(.5, uLeftSupport, uRightSupport);
--print("Step Body: L R LS RS B",uL[1],uR[1],uLeftSupport[1],uRightSupport[1],uB[1]);
  return se2_interpolate(.5, uLeftSupport, uRightSupport);
end

function set_velocity(vx, vy, vz)
  velCommand[3] = math.min(math.max(vz, velLimitA[1]), velLimitA[2]);
  local angle_turn=math.abs(velCommand[3]/velLimitA[2]); --0 to 1
--  local walkmag=math.min(1.0,Config.walk.rotlimit1-Config.walk.rotlimit2*angle_turn);  

  if math.abs(velCommand[3])>0.1 then   walkmag=0.8;
  else    walkmag=1;
  end

  velCommand[1] = math.min(math.max(vx, velLimitX[1]), velLimitX[2]);
  velCommand[2] = math.min(math.max(vy, velLimitY[1]), velLimitY[2]);
  velCommand[1] = velCommand[1]*walkmag;
  velCommand[2] = velCommand[2]*walkmag;
end

function force_set_velocity(vx, vy, vz)
  velCurrent[1],velCurrent[2],velCurrent[3]=vx,vy,vz;
  velCommand[1],velCommand[2],velCommand[3]=vx,vy,vz;
end

function zero_velocity()
  velCurrent[1],velCurrent[2],velCurrent[3]=0,0,0;
  velCommand[1],velCommand[2],velCommand[3]=0,0,0;
  uLeft = pose_global({-supportX, footY, 0},uBody);
  uRight = pose_global({-supportX, -footY, 0},uBody);
  t0 = Body.get_time();
  step.reset(uLeft,uBody,uRight,t0,starting_foot);
end


function update_velocity()
--Stepwise velocity change with maximum delta

  local velChange=vector.new({
	velCommand[1]-velCurrent[1],
	velCommand[2]-velCurrent[2],
	velCommand[3]-velCurrent[3]});


  velChange[1]=math.min(velDelta[1],math.max(-velDelta[1],velChange[1]));
  velChange[2]=math.min(velDelta[2],math.max(-velDelta[2],velChange[2]));
  velChange[3]=math.min(velDelta[3],math.max(-velDelta[3],velChange[3]));

--[[
  if Config.BodyFSM.level.speed==2 and velCommand[1]>0.04 then
     velChange[1]=math.min(0.01,math.max(-velDelta[1],velChange[1]));
  end
--]]

  velCurrent[1],velCurrent[2],velCurrent[3]=
	  velCurrent[1]+velChange[1],
	  velCurrent[2]+velChange[2],
	  velCurrent[3]+velChange[3];
end

function start()
  print("Walk start")
  stopRequest = false;
  initial_step=1;
  if (not active) then
    active = true;
    iStep0 = -1;
    t0 = Body.get_time();
    velCurrent[1],velCurrent[2], velCurrent[3]=0,0,0;
    step.stepqueue={};
  end
end

function stop()
  print("Walk stop request")
  stopRequest = true;
end


function doWalkKickLeft()
    t = Body.get_time();
    if t<tWalkKick+tWalkKickDelay then return; end
    tWalkKick=t;

    supportLeg=1 - step.supportLeg;

    local uLeft1 = step.uLeft2;
    local uRight1 = step.uRight2;
    local uBody1 = step.uBody2;
    local uSupport, uBody2;
    vel1={0.06,0,0};    vel2={0.12,0,0};

    --Hack for webots stability
    supportXWK={0 ,0};
    if tStep>0.3 then supportXWK = {0.05 ,0.02};end


    if supportLeg == 1 then  --Right support
	uLeft2[1],uLeft2[2],uLeft2[3]=uLeft1[1],uLeft1[2],uLeft1[3];
	uRight2[1],uRight2[2],uRight2[3]=uRight1[1],uRight1[2],uRight1[3];
	uSupport=pose_global({supportX,-supportY,0},uRight2);
	uBody2= se2_interpolate(.5, uLeft2,uRight2);
	step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);
	supportLeg=1-supportLeg;
    end

    uLeft2[1],uLeft2[2],uLeft2[3]=uLeft1[1],uLeft1[2],uLeft1[3];
    uRight2=pose_global(vel1,uRight1);
    uSupport=pose_global({supportX,supportY,0},uLeft2);
    uBody2= se2_interpolate(.7, uLeft2,uRight2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep*1.2,1);

    supportLeg=1-supportLeg;
    uLeft2=pose_global(vel2,uLeft1);
    uSupport=pose_global({supportX+supportXWK[1],-supportY,0},uRight2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    uBody2=pose_global({Config.walk.walkKickFrontComp,0,0},uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep*1.2,3);

    supportLeg=1-supportLeg;
    uRight2=pose_global(vel1,uRight2);
    uSupport=pose_global({supportX+supportXWK[2],supportY,0},uLeft2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);

    velCurrent[1],velCurrent[2],velCurrent[3]=0,0,0;
    velCommand[1],velCommand[2],velCommand[3]=0,0,0;
end

function doWalkKickRight()
    t = Body.get_time();
    if t<tWalkKick+tWalkKickDelay then return; end
    tWalkKick=t;

    supportLeg=1 - step.supportLeg;

    local uLeft1 = step.uLeft2;
    local uRight1 = step.uRight2;
    local uBody1 = step.uBody2;
    local uSupport, uBody2;

    vel1={0.06,0,0};    vel2={0.12,0,0};
    --Hack for webots stability
    supportXWK={0 ,0};
    if tStep>0.3 then supportXWK = {0.05, 0.02};end

    if supportLeg == 0 then  --Left support
	uLeft2[1],uLeft2[2],uLeft2[3]=uLeft1[1],uLeft1[2],uLeft1[3];
	uRight2[1],uRight2[2],uRight2[3]=uRight1[1],uRight1[2],uRight1[3];
        uSupport=pose_global({supportX,supportY,0},uLeft1);
        uBody2= se2_interpolate(.5, uLeft2,uRight2);
        step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);
	supportLeg=1-supportLeg;
    end

    --right support
    uRight2[1],uRight2[2],uRight2[3]=uRight1[1],uRight1[2],uRight1[3];
    uLeft2=pose_global(vel1,uLeft1);
    uSupport=pose_global({supportX,-supportY,0},uRight2);
    uBody2= se2_interpolate(.3, uLeft2,uRight2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep*1.2,1);

    --left support kick
    supportLeg=1-supportLeg;
    uRight2=pose_global(vel2,uRight1);
    uSupport=pose_global({supportX+supportXWK[1],supportY,0},uLeft2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    uBody2=pose_global({Config.walk.walkKickFrontComp,0,0},uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep*1.2,3);

    supportLeg=1-supportLeg;
    uLeft2=pose_global(vel1,uLeft2);
    uSupport=pose_global({supportX+supportXWK[2],-supportY,0},uRight2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);

    velCurrent[1],velCurrent[2],velCurrent[3]=0,0,0;
    velCommand[1],velCommand[2],velCommand[3]=0,0,0;
end



function doWalkKickSideRight()
--Use right foot, kick to left

    t = Body.get_time();
    if t<tWalkKick+tWalkKickDelay then return; end
    tWalkKick=t;

    supportLeg=1 - step.supportLeg;

    local uLeft1 = step.uLeft2;
    local uRight1 = step.uRight2;
    local uBody1 = step.uBody2;
    local uSupport, uBody2;

    vel1={0.04, -0.025, -0.4};    vel2={0,0.02,0.4}; vel3={-0,0.08,0.8};
    support1={0,0.01,0}; support2={0,0.03,0};
    support3={0.01+Config.walk.walkKickSideComp[1],
	      0.01+Config.walk.walkKickSideComp[2],0};

    if supportLeg == 1 then  --Right support
	uLeft2[1],uLeft2[2],uLeft2[3]=uLeft1[1],uLeft1[2],uLeft1[3];
	uRight2[1],uRight2[2],uRight2[3]=uRight1[1],uRight1[2],uRight1[3];
	uSupport=pose_global({supportX,-supportY,0},uRight2);
	uBody2= se2_interpolate(.5, uLeft2,uRight2);
	step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);
	supportLeg=1-supportLeg;
    else

    end

    uLeft2[1],uLeft2[2],uLeft2[3]=uLeft1[1],uLeft1[2],uLeft1[3];
    uRight2=pose_global(vel1,uRight1);
--    uSupport=pose_global({supportX,supportY,0},uLeft2);
    uSupport=pose_global({supportX+support1[1],-supportY+support1[2],0},uLeft2);
    uBody2= se2_interpolate(.7, uLeft2,uRight2);
    uBody2=pose_global(support1,uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,0.30,1);

    supportLeg=1-supportLeg;
    uLeft2=pose_global(vel2,uLeft1);
    uSupport=pose_global({supportX,-supportY,0},uRight2);
    uSupport=pose_global({supportX+support2[1],-supportY+support2[2],0},uRight2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    uBody2=pose_global(support2,uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,0.30,1);

    supportLeg=1-supportLeg;
    uRight2=pose_global(vel3,uRight2);
--    uSupport=pose_global({supportX,-supportY,0},uLeft2);
    uSupport=pose_global({supportX+support2[1],-supportY+support2[2],0},uLeft2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    uBody2=pose_global(support3,uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,0.30,3);


    supportLeg=1-supportLeg;
    uLeft2=pose_global({0,2*footY,0},uRight2);
    uSupport=pose_global({supportX,-supportY,0},uRight2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);

    velCurrent[1],velCurrent[2],velCurrent[3]=0,0,0;
    velCommand[1],velCommand[2],velCommand[3]=0,0,0;

end

function doWalkKickSideLeft()
--Use left foot, kick to right
    t = Body.get_time();
    if t<tWalkKick+tWalkKickDelay then return; end
    tWalkKick=t;

    supportLeg=1 - step.supportLeg;

    local uLeft1 = step.uLeft2;
    local uRight1 = step.uRight2;
    local uBody1 = step.uBody2;
    local uSupport, uBody2;

    vel1={0.04, 0.025, 0.4};    vel2={0,-0.02,-0.4}; vel3={-0,-0.08,-0.8};
    support1={0,-0.01,0}; support2={0,-0.03,0};support3={0.01,-0.01,0};

    support3={0.01+Config.walk.walkKickSideComp[1],
	      -0.01-Config.walk.walkKickSideComp[2],0};

    if supportLeg == 0 then  --Left support
	uLeft2[1],uLeft2[2],uLeft2[3]=uLeft1[1],uLeft1[2],uLeft1[3];
	uRight2[1],uRight2[2],uRight2[3]=uRight1[1],uRight1[2],uRight1[3];
        uSupport=pose_global({supportX,supportY,0},uLeft1);
        uBody2= se2_interpolate(.5, uLeft2,uRight2);
        step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);
	supportLeg=1-supportLeg;
    end

    --right support
    uRight2[1],uRight2[2],uRight2[3]=uRight1[1],uRight1[2],uRight1[3];
    uLeft2=pose_global(vel1,uLeft1);
    uSupport=pose_global({supportX+support1[1],-supportY+support1[2],0},uRight2);
    uBody2= se2_interpolate(.3, uLeft2,uRight2);
--    uBody2= se2_interpolate(.7, uLeft2,uRight2);
    uBody2=pose_global(support1,uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,0.30,1);


    --left support kick
    supportLeg=1-supportLeg;
    uRight2=pose_global(vel2,uRight1);
    uSupport=pose_global({supportX+support2[1],supportY+support2[2],0},uLeft2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    uBody2=pose_global(support2,uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,0.30,1);

    supportLeg=1-supportLeg;
    uLeft2=pose_global(vel3,uLeft2);
    uSupport=pose_global({supportX+support3[1],-supportY+support3[2],0},uRight2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    uBody2=pose_global(support3,uBody2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,0.30,3);

    supportLeg=1-supportLeg;
    uRight2=pose_global({0,-2*footY,0},uLeft2);
    uSupport=pose_global({supportX,supportY,0},uLeft2);
    uBody2= se2_interpolate(.5, uLeft2,uRight2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);

    velCurrent[1],velCurrent[2],velCurrent[3]=0,0,0;
    velCommand[1],velCommand[2],velCommand[3]=0,0,0;
end


function stopAlign()
    if active==false or stopRequest==true then return; end
    print("Walk stop request")
    stopRequest = true;

    local uLeft1 = step.uLeft2;
    local uRight1 = step.uRight2;
    local uBody1 = step.uBody2;
    local supportLeg=1 - step.supportLeg;

    local uSupport, uBody2;
    if supportLeg == 0 then  -- Left support
	uLeft2[1],uLeft2[2],uLeft2[3]=
		uLeft1[1],uLeft1[2],uLeft1[3];
    	uRight2 = pose_global({0,-2*footY,0},uLeft1);
        uSupport=pose_global({supportX,supportY,0},uLeft1);
    else  -- Right support
	uRight2[1],uRight2[2],uRight2[3]=
		uRight1[1],uRight1[2],uRight1[3];
    	uLeft2 = pose_global({0,2*footY,0},uRight1);
	uSupport=pose_global({supportX,-supportY,0},uRight1);
    end
    uBody2 = step_body(uLeft2, uRight2);
    step.enque(uLeft2,uBody2,uRight2,uSupport,supportLeg,tStep,1);
--    print("STEP ENQUE",uLeft2[1],uRight2[1],supportLeg);
    velCurrent[1],velCurrent[2],velCurrent[3]=0,0,0;
end


function get_odometry(u0)
  if (not u0) then
    u0 = vector.new({0, 0, 0});
  end
  local uFoot = se2_interpolate(.5, uLeft, uRight);
  return pose_relative(uFoot, u0), uFoot;
end

function get_body_offset()
  local uFoot = se2_interpolate(.5, uLeft, uRight);
  return pose_relative(uTorso, uFoot);
end


function pose_global(pRelative, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  return vector.new{pose[1] + ca*pRelative[1] - sa*pRelative[2],
                    pose[2] + sa*pRelative[1] + ca*pRelative[2],
                    pose[3] + pRelative[3]};
end

function pose_relative(pGlobal, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  local px = pGlobal[1]-pose[1];
  local py = pGlobal[2]-pose[2];
  local pa = pGlobal[3]-pose[3];
  return vector.new{ca*px + sa*py, -sa*px + ca*py, mod_angle(pa)};
end

function se2_interpolate(t, u1, u2)
  return vector.new{u1[1]+t*(u2[1]-u1[1]),
                    u1[2]+t*(u2[2]-u1[2]),
                    u1[3]+t*mod_angle(u2[3]-u1[3])};
end

