--Step code for NSL 2011
--Made by SJ, Dec 2010
--DO NOT DISTRIBUTE!

module(..., package.seeall);

require('Body')
require('Kinematics')
require('Config');
require('vector')
require('util')

mod_angle = util.mod_angle;
-- Walk Parameters
bodyHeight = Config.walk.bodyHeight;
bodyTilt=Config.walk.bodyTilt;
qLArm0=vector.new(Config.walk.qLArm);
qRArm0=vector.new(Config.walk.qRArm);
qLArm=vector.zeros(3);
qRArm=vector.zeros(3);
qLArm[1],qLArm[2],qLArm[3]=qLArm0[1],qLArm0[2],qLArm0[3];
qRArm[1],qRArm[2],qRArm[3]=qRArm0[1],qRArm0[2],qRArm0[3];

-- Walk Parameters
footX = Config.walk.footX+Config.walk.footXComp;
footY = Config.walk.footY+Config.walk.footYComp;
tStep = Config.walk.tStep;
tZmp = Config.walk.tZmp;
stepHeight = Config.walk.stepHeight;

supportCompL=Config.walk.supportCompL;
supportCompR=Config.walk.supportCompR;

hipRollCompensation = Config.walk.hipRollCompensation;
hipPitchCompensation = Config.walk.hipPitchCompensation;
anklePitchCompensation = Config.walk.anklePitchCompensation;
kneePitchCompensation = Config.walk.kneePitchCompensation;

hipPitchCompensation2 = Config.walk.hipPitchCompensation2;
ankleMod=Config.walk.ankleMod;
ankleMod2=Config.walk.ankleMod2;

bodyModZ=Config.walk.bodyModZ;
phBodyZ=Config.walk.phBodyZ;

--Feedback parameters
ankleImuParamX=Config.walk.ankleImuParamX;
ankleImuParamY=Config.walk.ankleImuParamY;
kneeImuParamX=Config.walk.kneeImuParamX;
hipImuParamY=Config.walk.hipImuParamY;
armImuParamX=Config.walk.armImuParamX;
armImuParamY=Config.walk.armImuParamY;

imuXenable=1;
imuYenable=1;

--Single support phases
ph1Single = Config.walk.phSingle[1];
ph2Single = Config.walk.phSingle[2];

--ZMP shift phases
ph1ZmpOrg = ph1Single;
ph2ZmpOrg = ph2Single;

--Stabilization variables
uSupportShift=vector.new({0, 0});
uTorsoShift = vector.new({0, 0, 0});
ankleShift = vector.new({0, 0});
armShift=vector.new({0,0});
kneeShift=0;
hipShift=vector.new({0,0});

--Flex compensation variables
qLHipRollCompensation = 0;
qRHipRollCompensation = 0;
qLHipPitchCompensation = 0;	
qRHipPitchCompensation = 0;
qAnkleMod=vector.zeros(2);

--Foot and torso poses
uLeft=vector.zeros(3);
uRight=vector.zeros(3);
uBody=vector.zeros(3);

uLeft1=vector.zeros(3);
uRight1=vector.zeros(3);
uBody1=vector.zeros(3);

uLeft2=vector.zeros(3);
uRight2=vector.zeros(3);
uBody2=vector.zeros(3);

uTorso=vector.zeros(3);
uZmp=vector.zeros(3);
uSupport=vector.zeros(3);

pLLeg = vector.zeros(6);
pRLeg = vector.zeros(6);
pTorso = vector.zeros(6);

ph=0;
ph0=0;
phSingle=0;
tStart=0;
tStep=0;
supportLeg=0;
has_ball=0;

walkphase=0; --0 for DS1, 1 for SS, 2 for DS2, 3 for DS_STOP 

--Step queue
stepqueue={}
count=0;

function reset(uL,uB,uR, t, sL)
   uLeft[1],uLeft[2],uLeft[3]=uL[1],uL[2],uL[3];
   uRight[1],uRight[2],uRight[3]=uR[1],uR[2],uR[3];
   uBody[1],uBody[2],uBody[3]=uB[1],uB[2],uB[3];
   uZmp[1],uZmp[2],uZmp[3]=uB[1],uB[2],uB[3];
   uSupport[1],uSupport[2],uSupport[3]=uB[1],uB[2],uB[3];

   uLeft1[1],uLeft1[2],uLeft1[3]=uL[1],uL[2],uL[3];
   uRight1[1],uRight1[2],uRight1[3]=uR[1],uR[2],uR[3];
   uBody1[1],uBody1[2],uBody1[3]=uB[1],uB[2],uB[3];

   uLeft2[1],uLeft2[2],uLeft2[3]=uL[1],uL[2],uL[3];
   uRight2[1],uRight2[2],uRight2[3]=uR[1],uR[2],uR[3];
   uBody2[1],uBody2[2],uBody2[3]=uB[1],uB[2],uB[3];

   tStep=0;
   tStart=t-tStep;
   supportLeg=1-sL; 
   stepqueue={};
end

function enque(uL2,uB2,uR2,uS,sLeg,tStep,walktype)
   local newStep={}
   newStep.uLeft2=vector.new(uL2);
   newStep.uRight2=vector.new(uR2);
   newStep.uBody2=vector.new(uB2);
   newStep.supportLeg=sLeg;
   newStep.uSupport=vector.new(uS);
   newStep.tStep=tStep;
   newStep.walktype=walktype;
   stepqueue[#stepqueue+1]=newStep;
end

function deque(t)
  local newStep=stepqueue[1];

  uLeft[1],uLeft[2],uLeft[3]=uLeft2[1],uLeft2[2],uLeft2[3];
  uRight[1],uRight[2],uRight[3]=uRight2[1],uRight2[2],uRight2[3];
  uBody[1],uBody[2],uBody[3]=uBody2[1],uBody2[2],uBody2[3];
  uZmp[1],uZmp[2],uZmp[3]=uBody2[1],uBody2[2],uBody2[3];

  uLeft1[1],uLeft1[2],uLeft1[3]=uLeft2[1],uLeft2[2],uLeft2[3];
  uRight1[1],uRight1[2],uRight1[3]=uRight2[1],uRight2[2],uRight2[3];
  uBody1[1],uBody1[2],uBody1[3]=uBody2[1],uBody2[2],uBody2[3];

  uLeft2[1],uLeft2[2],uLeft2[3]=
	newStep.uLeft2[1],newStep.uLeft2[2],newStep.uLeft2[3];
  uRight2[1],uRight2[2],uRight2[3]=
	newStep.uRight2[1],newStep.uRight2[2],newStep.uRight2[3];
  uBody2[1],uBody2[2],uBody2[3]=
	newStep.uBody2[1],newStep.uBody2[2],newStep.uBody2[3];

  supportLeg=newStep.supportLeg;
  uSupport[1],uSupport[2],uSupport[3] = newStep.uSupport[1],newStep.uSupport[2],newStep.uSupport[3];
  walktype=newStep.walktype;

--print(string.format("walk deque: L0 %.4f B0 %.4f R0 %.4f -> L1 %.4f B1 %.4f R1 %.4f, S:%.4f",
--	uLeft1[1],uBody1[1],uRight1[1],uLeft2[1],uBody2[1],uRight2[1],uSupport[1]));

  if supportLeg == 0 then --left support
     uSupport = pose_global(supportCompL, uSupport);
  else  -- Right support
     uSupport = pose_global(supportCompR, uSupport);
  end   
  tStart=t;
  tStep=newStep.tStep;
  table.remove(stepqueue,1);
end

function isactive(t)
  if t<=tStart+tStep or #stepqueue>0 then return true;
  else return false;  end
end

function update(t)
  count=count+1;
  if t>tStart+tStep then --deque one step
     if #stepqueue ==0 then
	walkphase=3; --0 for DS1, 1 for SS, 2 for DS2, 3 for DS_STOP 
	tStart=t-tStep;
        joint_feedback();
	phComp=0; --DS
        motion_legs();
        if has_ball==0 then  motion_arms();end
	return; 
     else
	deque(tStart+tStep);
	Body.set_lleg_hardness(1);
	Body.set_rleg_hardness(1);
        ph0=0;
     end
  end

  if isactive(t) then             --Executing step
     ph= (t-tStart)/tStep;
     update_legs();
     flex_compensation();
     joint_feedback();
     update_body();
     ph0=ph;
     motion_legs();
     if has_ball==0 then  motion_arms();end
  else
--     print("BUG: NONACTIVE STEP",t,tStart+tStep,#stepqueue)
  end
end

function foot_phase(ph)
  -- Computes relative x,z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0

  if walktype==2 then
	  phSingle = math.min(math.max(ph-0.2, 0)/(0.8-0.2),1);
	  phComp=math.min(1, phSingle/.1, (1-phSingle)/.1);
  else	--normal walk
	  phSingle = math.min(math.max(ph-ph1Single, 0)/(ph2Single-ph1Single),1);
	  phComp=math.min(1, phSingle/.1, (1-phSingle)/.1);
  end



  --0 for DS1, 1 for SS, 2 for DS2, 3 for DS_STOP 
  if ph<ph1Single then walkphase=0;
  elseif ph<ph2Single then walkphase=1;
  else walkphase=2;
  end


  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle);
  local xf = .5*(1-math.cos(math.pi*phSingleSkew));
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew));

  if walktype==2 then
	zf=0;

--	walkphase=3;
  elseif walktype==3 then
        zf=zf*2.0;
	local kickN=3;
	if ph<0.5 then xf=kickN*phSingle;
	else xf=(2-kickN)*(phSingle-0.5) + kickN/2;
	end
  end
  return xf, zf;
end

function flex_compensation()
  qLHipRollCompensation , qRHipRollCompensation = 0,0;
  qLHipPitchCompensation , qRHipPitchCompensation = 0,0;
  qLAnklePitchCompensation , qRAnklePitchCompensation = 0,0;
  qLKneePitchCompensation , qRKneePitchCompensation = 0,0;
  ankleShift[1],ankleShift[2]=0,0;
  kneeShift=0;
  hipShift[1],hipShift[2]=0,0;
  qAnkleMod[1],qAnkleMod[2]=0,0;


  local phSingle2= math.max(0,phSingle*2-1);
  local phCompAnkle=math.min(1, phSingle2/.1, (1-phSingle2)/.1);

  if supportLeg == 0 then -- Left support
      qLHipRollCompensation = hipRollCompensation*phComp;
      qLHipPitchCompensation = hipPitchCompensation*phComp;
      qLAnklePitchCompensation = anklePitchCompensation*phComp;
      qLKneePitchCompensation = kneePitchCompensation*phComp;

      --Ankle angle modulation so that robot won't kick the ground

      local uRight2Body=pose_relative(uRight2,uBody);
      local uRight1Body=pose_relative(uRight1,uBody);

      vForward=uRight2Body[1]-uRight1Body[1];
      vLeft=uRight2Body[2]-uRight1Body[2];
      if vForward>0 then
         qAnkleMod[1],qAnkleMod[2]=
  	    ankleMod[1]*vForward*phComp,ankleMod[2]*vLeft*phCompAnkle;
      else
	 qAnkleMod[1],qAnkleMod[2]=
	    ankleMod2*vForward*phComp,ankleMod[2]*vLeft*phCompAnkle;
      end

      --TODO: Fix this
      if uRight2[1]<uRight1[1] then --when going backwards
	 qLHipPitchCompensation = hipPitchCompensation2*phComp;
      end
  else	--Right support
      qRHipRollCompensation = -hipRollCompensation*phComp;
      qRHipPitchCompensation = hipPitchCompensation*phComp;
      qRAnklePitchCompensation = anklePitchCompensation*phComp;
      qRKneePitchCompensation = kneePitchCompensation*phComp;


      --Ankle angle modulation so that robot won't kick the ground
      local uLeft2Body=pose_relative(uLeft2,uBody);
      local uLeft1Body=pose_relative(uLeft1,uBody);

      vForward=uLeft2Body[1]-uLeft1Body[1];
      vLeft=uLeft2Body[2]-uLeft1Body[2];

      if vForward>0 then
         qAnkleMod[1],qAnkleMod[2]=
  	    ankleMod[1]*vForward*phComp,ankleMod[2]*vLeft*phComp;
      else
	 qAnkleMod[1],qAnkleMod[2]=
	    ankleMod2*vForward*phComp,ankleMod[2]*vLeft*phComp;
      end

      if uRight2[1]<uRight1[2] then
	 qRHipPitchCompensation = hipPitchCompensation2*phComp;
      end
  end
end

function joint_feedback()
--Ankle stabilization using gyro feedback
  imuGyr = Body.get_sensor_imuGyr();
  gyro_roll,gyro_pitch=imuGyr[1],imuGyr[2];


  if imuXenable==0 then gyro_pitch=0; end
  if imuYenable==0 then gyro_roll=0; end

  --print(gyro_roll,gyro_pitch)

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

end

function bodyZ(ph)
   if ph<phBodyZ[1] then
	ret=ph*ph/phBodyZ[1]/phBodyZ[1]*bodyModZ[2] + bodyModZ[1];
   elseif ph>phBodyZ[2] then
	ret=(1-ph)*(1-ph)/(1-phBodyZ[2])/(1-phBodyZ[2])*bodyModZ[2] + bodyModZ[1];
   else	
	ret=bodyModZ[2]+bodyModZ[3]-
	    bodyModZ[3]* (ph-(phBodyZ[1]+phBodyZ[2])/2 ) * (ph- (phBodyZ[1]+phBodyZ[2])/2 )
			  /(  (phBodyZ[1]-phBodyZ[2])/2)  /( (phBodyZ[1]-phBodyZ[2])/2)
	    +bodyModZ[1];
   end
   return ret;
end

function motion_legs()

  pLLeg[1], pLLeg[2], pLLeg[6] = uLeft[1], uLeft[2], uLeft[3];
  pRLeg[1], pRLeg[2], pRLeg[6] = uRight[1], uRight[2], uRight[3];
  uTorso=pose_global(vector.new({-footX,0,0}),uBody);

  if walktype==3 then
	qAnkleMod={0,0};
  end

  --Head angle affects walk balance
  headAngles = Body.get_head_position();
  --1: head down 0: head up
  headPitch=(headAngles[2]+40*math.pi/180)/(60*math.pi/180);
  uTorsoShiftHead = headPitch * Config.walk.headPitchFactor;

  --Carrying something
  local bodyTiltOffset=0;
  if (has_ball==1 and Config.gametype~='stretcher') then -- With stretcher, we don't want to mess with our body tilt
    uTorso=pose_global(vector.new({-footX-0.02,0,0}),uBody);
    bodyTiltOffset=-5*math.pi/180;
  end

  pTorso[1], pTorso[2], pTorso[3], pTorso[6], pTorso[5],pTorso[4]  = 
     uTorso[1]+uTorsoShift[1]+uTorsoShiftHead, uTorso[2]+uTorsoShift[2], bodyHeight, 
     uTorso[3], bodyTilt+bodyTiltOffset,0;

  pTorso[3]=pTorso[3]+bodyZ(ph);

  qLegs = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);

--  if phComp==0 then --Double support
  if walkphase==3 then --DS and stop
 	qLegs[2] = qLegs[2] + qLHipRollCompensation+hipShift[2];
  	qLegs[8] = qLegs[8] + qRHipRollCompensation+hipShift[2];

  	qLegs[3] = qLegs[3] + qLHipPitchCompensation;
  	qLegs[9] = qLegs[9] + qRHipPitchCompensation;

  	qLegs[4] = qLegs[4] + kneeShift;
  	qLegs[10] = qLegs[10] + kneeShift;

  	qLegs[5] = qLegs[5]  + ankleShift[1];
  	qLegs[11] = qLegs[11]  + ankleShift[1];

  	qLegs[6] = qLegs[6] + ankleShift[2];
  	qLegs[12] = qLegs[12] + ankleShift[2];
  elseif  supportLeg == 0 then -- Left support
  	qLegs[2] = qLegs[2] + qLHipRollCompensation+hipShift[2];
  	qLegs[8] = qLegs[8] + qRHipRollCompensation+hipShift[2];

  	qLegs[3] = qLegs[3] + qLHipPitchCompensation;
  	qLegs[4] = qLegs[4] + qLKneePitchCompensation+kneeShift;

  	qLegs[10] = qLegs[10] + kneeShift;
  	qLegs[11] = qLegs[11]  + ankleShift[1];

  	qLegs[5] = qLegs[5]  + qLAnklePitchCompensation+ankleShift[1];
  	qLegs[11] = qLegs[11]  + qAnkleMod[1];

  	qLegs[6] = qLegs[6] + Config.walk.ankleFactor*ankleShift[2];
  	qLegs[12] = qLegs[12] + qAnkleMod[2]+ankleShift[2];

  else	--Right support
  	qLegs[2] = qLegs[2] + qLHipRollCompensation+hipShift[2];
  	qLegs[8] = qLegs[8] + qRHipRollCompensation+hipShift[2];
  	qLegs[9] = qLegs[9] + qRHipPitchCompensation;
  	qLegs[10] = qLegs[10] + qRKneePitchCompensation+kneeShift;

  	qLegs[4] = qLegs[4] + kneeShift;
  	qLegs[5] = qLegs[5]  + ankleShift[1];

  	qLegs[5] = qLegs[5]  + qAnkleMod[1];
  	qLegs[11] = qLegs[11]  + qRAnklePitchCompensation+ankleShift[1];

  	qLegs[6] = qLegs[6] + qAnkleMod[2]+ankleShift[2];
  	qLegs[12] = qLegs[12] + Config.walk.ankleFactor*ankleShift[2];

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

function update_legs()
  xFoot, zFoot = foot_phase(ph);
  if supportLeg == 0 then -- Left support
    uRight = se2_interpolate(xFoot, uRight1, uRight2);
    pLLeg[3],pRLeg[3] = 0, stepHeight*zFoot;
  else    -- Right support
    uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
    pLLeg[3], pRLeg[3] = stepHeight*zFoot, 0;
  end
end

function update_body()
  if walktype==1 or walktype==2 then
     uBody[1],uZmp[1]=zmp_solve(uZmp[1],uSupport[1]+uSupportShift[1],
	uBody2[1],uBody[1],uBody2[1],ph0,ph);
     uBody[2],uZmp[2]=zmp_solve(uZmp[2],uSupport[2]+uSupportShift[2],
	uBody2[2],uBody[2],uBody2[2],ph0,ph);
     uBody[3]=0.5*(uLeft[3]+uRight[3]);   
  elseif walktype==3 then
     uBody=se2_interpolate(ph,uBody1,uBody2);
     uZmp[1],uZmp[2],uZmp[3]=uBody[1],uBody[2],uBody[3];
  end
end

function zmp_solve(z0,z1,z2,x,x2,ph0,ph)
--[[
   Solves ZMP equation using current COM pos x0 and ZMP pos z0
   x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
   where the ZMP point is piecewise linear:
   z(ph0*tStep) = z0, z(T1 < t < T2) = z1, z(tStep) = z2
--]]
   ph1Zmp=ph1ZmpOrg;
   ph2Zmp=ph2ZmpOrg;

   local T0 = tStep*ph0;
   local T = tStep*ph;
   local T1 = tStep*ph1Zmp;
   local T2 = tStep*ph2Zmp; 
   local T3 = tStep;
   local expTStep = math.exp(T3/tZmp);
   local expT0 = math.exp(T0/tZmp);
   local expT = math.exp(T/tZmp);
   
   if (ph<ph1Zmp) then
        local m1 = (z1-z0)/(T1-T0);
        local m2 = (z2-z1)/(T3-T2);                 
        local c1 = x  - z0 + tZmp*m1*math.sinh((T0-T1)/tZmp);
        local c2 = x2 - z2 + tZmp*m2*math.sinh((T3-T2)/tZmp);
        local aP = (c2/expT0 - c1/expTStep) / (expTStep/expT0-expT0/expTStep);
        local aN = (c1*expTStep - c2*expT0) / (expTStep/expT0-expT0/expTStep);
        local z = z0 + (z1-z0)*(ph-ph0)/(ph1Zmp-ph0);
        local c = z + aP*expT+aN/expT - tZmp*m1*math.sinh((T-T1)/tZmp);
        return c,z;
   elseif (ph<ph2Zmp) then        
        local m2 = (z2-z1)/(T3-T2);                 
        local c1 = x -z1;
        local c2 = x2-z2 + tZmp*m2*math.sinh((T3-T2)/tZmp);
        local aP = (c2/expT0 - c1/expTStep) / (expTStep/expT0-expT0/expTStep);
        local aN = (c1*expTStep - c2*expT0) / (expTStep/expT0-expT0/expTStep);
        local z = z1;
        local c = z + aP*expT+aN/expT;
        return c,z;
   else        
        local m2 = (z2-z0)/(T3-T0);                 
        local c1 = x  - z0 + tZmp*m2*math.sinh((T0-T2)/tZmp);
        local c2 = x2 - z2 + tZmp*m2*math.sinh((T3-T2)/tZmp);
        local aP = (c2/expT0 - c1/expTStep) / (expTStep/expT0-expT0/expTStep);
        local aN = (c1*expTStep - c2*expT0) / (expTStep/expT0-expT0/expTStep);
        local z = z0 + (z2-z0)*(ph-ph0)/(1-ph0);
        local c = z + aP*expT+aN/expT - tZmp*m2*math.sinh((T-T2)/tZmp);
        return c,z;
   end
end

function getPos()
  return uLeft,uRight,uBody,uTorso,uZmp;
end

function procFunc(a,deadband,maxvalue)
   if a>0 then b=math.min( math.max(0,math.abs(a)-deadband), maxvalue);
   else b=-math.min( math.max(0,math.abs(a)-deadband), maxvalue);
   end
   return b;
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

function getPhase( t )
  return t - tStart;
end
