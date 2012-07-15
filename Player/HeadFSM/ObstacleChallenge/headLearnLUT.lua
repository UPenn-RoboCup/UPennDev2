module(..., package.seeall);

require('Body')
require('vcm')
require('mcm')
require('ColorLUT')

t0 = 0;
timeout = 6.0;

enable_learning = 1

function entry()
  pitchBias =  mcm.get_headPitchBias();--robot specific head bias

  print(_NAME.." entry");

  if vcm.get_camera_learned_new_lut() == 1 then
    return 'done'
  end

  t0 = Body.get_time();

  -- set head to default position
  local yaw = 0;
  local pitch = 50*math.pi/180;

  print(yaw, pitch)
  Body.set_head_command({yaw, pitch-pitchBias});

--  ColorLUT.learn_lut_from_mask()

end

function update()
  headAngle = Body.get_head_position();
  if (headAngle[2]*180/math.pi - 50) < 1 and enable_learning == 0 then
    enable_learning = 1;
    ColorLUT.learn_lut_from_mask();
  end

  local t = Body.get_time()
  
  if (t - t0) > timeout then
    return 'timeout'
  end

  local flag = vcm.get_camera_learned_new_lut();
  if flag == 1 then
    return 'done'
  end

end

function exit()
end

