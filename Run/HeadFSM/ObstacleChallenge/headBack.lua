------------------------------
--NSL Linear two-line head scan
------------------------------

module(..., package.seeall);

require('Body')
require('wcm')
require('mcm')
require('ocm')

pitch0 = 43*math.pi/180;
pitchMag = 15.5*math.pi/180;
yawMag = 10*math.pi/180;
yaw0 = 0*math.pi/180;

tScan = 3.0;
timeout = tScan;

t0 = 0;
direction = 1;


function entry()
  print("Head SM:".._NAME.." entry");

  ocm.set_vision_update(0);
  -- start scan in ball's last known direction
  t0 = Body.get_time();
  ball = wcm.get_ball();
--  timeout = tScan * 2;

  yaw_0, pitch_0 = HeadTransform.ikineCam(ball.x, ball.y,0);
  local currentYaw = Body.get_head_position()[1];
  InitHeadAngle = Body.get_head_position();
--  print(InitHeadAngle[1] * 180 / math.pi, InitHeadAngle[2] * 180 / math.pi);

  if currentYaw>0 then
    direction = 1;
    yawDir = 1
  else
    direction = -1;
    yawDir = -1
  end
--[[
  if pitch_0>pitch0 then
    pitchDir=1;
  else
    pitchDir=-1;
  end
  --]]
end

function update()
  pitchBias =  mcm.get_headPitchBias();--Robot specific head angle bias

  local t = Body.get_time();
  -- update head position

  local ph = (t-t0)/tScan;
  ph = ph - math.floor(ph);

  yaw = InitHeadAngle[1] - ph * InitHeadAngle[1];
  pitch = InitHeadAngle[2] - ph * InitHeadAngle[2];
--  print(ph, yaw * 180 / math.pi, pitch * 180/ math.pi);

  Body.set_head_command({yaw, pitch});

  if (t - t0 > timeout) then
    return 'timeout';
  end
end

function exit()
end
