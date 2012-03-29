module(..., package.seeall);

require('Body')
require('vcm')

t0 = 0;
timeout = 1.0;

function entry()
  pitchBias=vcm.get_camera_pitchBias(); --Robot specific head angle bias

  print(_NAME.." entry");

  t0 = Body.get_time();

  -- set head to default position
  local yaw = 0;
  local pitch = 20*math.pi/180;

  Body.set_head_command({yaw, pitch-pitchBias});

  -- continuously switch cameras
  vcm.set_camera_command(-1);
end

function update()
end

function exit()
end
