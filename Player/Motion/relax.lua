module(..., package.seeall);

require('Body')

t0 = 0;
timeout = 1.0;

footX = Config.walk.footX or 0;
footY = Config.walk.footY;
supportX = Config.walk.supportX;
pLLeg = vector.new({-supportX , footY, 0, 0,0,0});
pRLeg = vector.new({-supportX , -footY, 0, 0,0,0});

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  Body.set_body_hardness(0);
  Body.set_syncread_enable(1);
end

function update()
  local t = Body.get_time();

  local qSensor = Body.get_sensor_position();
  Body.set_actuator_command(qSensor);

  --update vcm body information
  local qLLeg = Body.get_lleg_position();
  local qRLeg = Body.get_rleg_position();
  local dpLLeg = Kinematics.torso_lleg(qLLeg);
  local dpRLeg = Kinematics.torso_rleg(qRLeg);

  pTorsoL=pLLeg+dpLLeg;
  pTorsoR=pRLeg+dpRLeg;
  pTorso=(pTorsoL+pTorsoR)*0.5;

  vcm.set_camera_bodyHeight(pTorso[3]);
  vcm.set_camera_bodyTilt(pTorso[5]);

  if (t - t0 > timeout) then
    return "timeout";
  end
end

function exit()
end
