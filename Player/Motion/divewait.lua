module(..., package.seeall);
name = ...;

require('Config')
require('vector')
require('Kinematics')
require('Body')
require('walk')

active = true;
sitdone=false;

t0 = 0;

--bodyHeight = Config.sit.bodyHeight;
bodyHeight = 0.25;

supportX = Config.sit.supportX or  0.0;
footY = Config.walk.footY;
bodyTilt = Config.walk.bodyTilt;

-- pTorso fixed for stance:
pTorso = vector.new({supportX, 0, bodyHeight, 0,0,0});

-- Final stance foot position6D
pLLegStance = vector.new({0, footY, 0, 0,0,0});
pRLegStance = vector.new({0, -footY, 0, 0,0,0});

-- Max change in postion6D to reach stance:
dpLimit=vector.new({.2,.06,.06,.2,.6,.2});

qLArm=math.pi/180*vector.new({0,90,-0});
qRArm=math.pi/180*vector.new({0,-90,-0});

qLArm=math.pi/180*vector.new({80,20,0});
qRArm=math.pi/180*vector.new({80,-20,0});


function entry()
  print("Motion SM:".._NAME.." entry");

  started=false;
  Body.set_head_command({0,0});
  Body.set_head_hardness(.5);
  Body.set_larm_hardness(.1);
  Body.set_rarm_hardness(.1);
  walk.stop();
  Body.set_syncread_enable(1);
  t0=Body.get_time();

  sitdone=false;
end

function update()
  --Wait until walking is ended
  if walk.active then
     walk.update();
     t0=Body.get_time();
     return
  end
  local t = Body.get_time();
  local dt = t - t0;
  if not started then
    if dt>0.2 then
--	if true then
      local qSensor = Body.get_sensor_position();
      local dpLLeg = Kinematics.lleg_torso(Body.get_lleg_position());
      local dpRLeg = Kinematics.rleg_torso(Body.get_rleg_position());
      pLLeg = pTorso + dpLLeg;
      pRLeg = pTorso + dpRLeg;
--    Body.set_actuator_command(qSensor);
      Body.set_syncread_enable(0);
      Body.set_lleg_hardness(.7);
      Body.set_rleg_hardness(.7);
      started=true;
    else 
      Body.set_syncread_enable(1);
    return;
    end
  end

  if sitdone then return; end

  Body.set_larm_command(qLArm)
  Body.set_rarm_command(qRArm)

  t0 = t;

  local tol = true;
  local tolLimit = 1e-8;
  dpDeltaMax = dt*dpLimit;
  dpLeft = pLLegStance - pLLeg;
  for i = 1,6 do
    if (math.abs(dpLeft[i]) > tolLimit) then
      tol = false;
      if (dpLeft[i] > dpDeltaMax[i]) then
        dpLeft[i] = dpDeltaMax[i];
      elseif (dpLeft[i] < -dpDeltaMax[i]) then
        dpLeft[i] = -dpDeltaMax[i];
      end
    end
  end
  pLLeg = pLLeg + dpLeft;
	 
  dpRight = pRLegStance - pRLeg;
  for i = 1,6 do
    if (math.abs(dpRight[i]) > tolLimit) then
      tol = false;
      if (dpRight[i] > dpDeltaMax[i]) then
        dpRight[i] = dpDeltaMax[i];
      elseif (dpRight[i] < -dpDeltaMax[i]) then
        dpRight[i] = -dpDeltaMax[i];
      end
    end
  end
  pRLeg = pRLeg + dpRight;

  q = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, 0);
  Body.set_lleg_command(q);

  sitdone=tol;
end

function exit()
end
