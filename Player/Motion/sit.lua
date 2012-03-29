module(..., package.seeall);
name = ...;

require('Config')
require('vector')
require('Kinematics')
require('Body')
require('walk')

active = true;
t0 = 0;

footX = Config.walk.footX or 0;
footY = Config.walk.footY;
supportX = Config.walk.supportX;

footXSit = Config.stance.footXSit or 0;
bodyHeightSit = Config.stance.bodyHeightSit;
bodyTiltSit = 0;

-- Final stance foot position6D
pTorsoTarget = vector.new({-footXSit, 0, bodyHeightSit, 0,bodyTiltSit,0});
pLLeg = vector.new({-supportX, footY, 0, 0,0,0});
pRLeg = vector.new({-supportX, -footY, 0, 0,0,0});

-- Max change in postion6D to reach stance:
dpLimit=Config.stance.dpLimitSit or vector.new({.1,.01,.03,.1,.3,.1});

tFinish=0;
tStart=0;

function entry()
  print("Motion SM:".._NAME.." entry");

  walk.stop();
  started=false;
  Body.set_head_command({0,0});
  Body.set_head_hardness(.5);
  Body.set_larm_hardness(.1);
  Body.set_rarm_hardness(.1);
  t0=Body.get_time();
end

function update()
  local t = Body.get_time();
  if walk.active then
     walk.update();
     t0=Body.get_time();
     return;
  end

  local dt = t - t0;
  if not started then 
    started=true;

    --Now we assume that the robot always start sitting from stance position
    pTorso = vector.new({-footX,0,vcm.get_camera_bodyHeight(),
	  	         0,vcm.get_camera_bodyTilt(),0});
    pLeft = vector.new({-supportX,footY,0,0,0,0});
    pRight= vector.new({-supportX,-footY,0,0,0,0});
    Body.set_lleg_command(qLLeg);
    Body.set_rleg_command(qRLeg);
    Body.set_lleg_hardness(1);
    Body.set_rleg_hardness(1);
    t0 = Body.get_time();
    tStart=t;
    count=1;
  end

  t0 = t;
  local tol = true;
  local tolLimit = 1e-6;
  dpDeltaMax = dt*dpLimit;

  dpTorso = pTorsoTarget - pTorso;
  for i = 1,6 do
    if (math.abs(dpTorso[i]) > tolLimit) then
      tol = false;
      if (dpTorso[i] > dpDeltaMax[i]) then
        dpTorso[i] = dpDeltaMax[i];
      elseif (dpTorso[i] < -dpDeltaMax[i]) then
        dpTorso[i] = -dpDeltaMax[i];
      end
    end
  end

  pTorso=pTorso+dpTorso;

  vcm.set_camera_bodyHeight(pTorso[3]);
  vcm.set_camera_bodyTilt(pTorso[5]);
  q = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, 0);
  Body.set_lleg_command(q);

  if (tol) then
    print("Sit done, time elapsed",t-tStart)
    return "done"
  end

end

function exit()
end
